using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Plugin;

namespace ThrottleControlPlugin
{
    public class ThrottleControlPlugin : MissionPlanner.Plugin.Plugin
    {
        private Form _controlForm;

        public override string Name { get { return "四电机油门控制器"; } }
        public override string Version { get { return "1.1"; } }
        public override string Author { get { return "Your Name"; } }

        public override bool Init() { return true; }

        public override bool Loaded()
        {
            _controlForm = new ThrottleControlForm();
            _controlForm.Show(Host.MainForm);
            return true;
        }

        public override bool Exit()
        {
            if (_controlForm != null) { _controlForm.Close(); _controlForm.Dispose(); }
            return true;
        }

        public override bool Loop() { return true; }
    }

    public class ThrottleControlForm : Form
    {
        // ===================== 原有: 电机油门控制 =====================
        private TrackBar[] trackBars = new TrackBar[4];
        private NumericUpDown[] numericUpDowns = new NumericUpDown[4];
        private Label[] labels = new Label[4];
        private Button btnSyncAll, btnEmergencyStop;
        private System.Windows.Forms.Timer _debounceTimer;
        private int[] _pendingValues = new int[4];
        private bool[] _pendingChannels = new bool[4];
        private DateTime[] _lastSendTime = new DateTime[4];
        private bool _suppressEvents;
        private float[] _receivedServo = new float[4];
        private Label[] _rxValueLabels = new Label[4];
        private Label _rxStatusLabel;
        private System.Windows.Forms.Timer _rxRefreshTimer;
        private System.Windows.Forms.Timer _streamReqTimer;
        private object _rxLock = new object();
        private int _totalPacketCount;
        private int _servoPacketCount;
        private int _rcPacketCount;
        private bool _hasServoData;
        private string _msgSource = "";
        private Dictionary<uint, int> _msgIdCounts = new Dictionary<uint, int>();

        // ===================== 新增: 姿态/角速度目标控制 =====================
        private TrackBar[] _angleTbs = new TrackBar[3];
        private NumericUpDown[] _angleNuds = new NumericUpDown[3];
        private Label[] _angleLbls = new Label[3];
        private Label[] _angleDegLbls = new Label[3];

        private TrackBar[] _gyroTbs = new TrackBar[3];
        private NumericUpDown[] _gyroNuds = new NumericUpDown[3];
        private Label[] _gyroLbls = new Label[3];

        private DateTime _lastTargetSend = DateTime.MinValue;
        private bool _targetSuppress;
        private const int TARGET_SEND_INTERVAL_MS = 50;

        private static readonly string[] AngleAxisNames = { "Roll", "Pitch", "Yaw" };
        private static readonly string[] GyroAxisNames = { "X", "Y", "Z" };

        private const float ANGLE_MAX_RAD = 3.15f;
        private const float GYRO_MAX_RAD = 34.9f;
        private const int ANGLE_TB_MAX = 628;    // ±3.14 rad → 628 steps (0.01 rad each)
        private const int GYRO_TB_MAX = 698;     // ±34.9 rad/s → 698 steps (0.1 rad/s each)

        private const int MSG_ID_DEBUG_FLOAT_ARRAY = 350;

        public ThrottleControlForm()
        {
            InitializeComponents();
            InitializeTargetControls();
            this.Height = 480;

            _debounceTimer = new System.Windows.Forms.Timer { Interval = 50 };
            _debounceTimer.Tick += OnDebounceTick;
            SubscribeMavlink();
            _rxRefreshTimer = new System.Windows.Forms.Timer { Interval = 200 };
            _rxRefreshTimer.Tick += OnRxRefresh;
            _rxRefreshTimer.Start();

            _streamReqTimer = new System.Windows.Forms.Timer { Interval = 3000 };
            _streamReqTimer.Tick += RequestServoStream;
            _streamReqTimer.Start();
        }

        // ===================== 原有: 电机油门 UI =====================
        private void InitializeComponents()
        {
            this.Text = "四电机油门控制器";
            this.Size = new Size(870, 320);
            this.StartPosition = FormStartPosition.CenterScreen;
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.BackColor = Color.FromArgb(45, 45, 48);

            for (int i = 0; i < 4; i++)
            {
                int idx = i;
                int x = 20 + i * 200;
                int y = 30;

                labels[i] = new Label
                {
                    Text = string.Format("电机 {0}", i + 1),
                    Location = new Point(x, y),
                    Size = new Size(180, 25),
                    Font = new Font("微软雅黑", 11, FontStyle.Bold),
                    ForeColor = Color.White,
                    TextAlign = ContentAlignment.MiddleCenter
                };

                trackBars[i] = new TrackBar
                {
                    Minimum = 0,
                    Maximum = 4095,
                    Value = 0,
                    Location = new Point(x, y + 30),
                    Size = new Size(180, 45),
                    Orientation = Orientation.Horizontal,
                    TickFrequency = 500,
                    LargeChange = 200,
                    SmallChange = 50,
                    BackColor = Color.FromArgb(60, 60, 65),
                    TickStyle = TickStyle.BottomRight
                };
                trackBars[idx].Scroll += (s, e) => OnThrottleChanged(idx, trackBars[idx].Value);

                numericUpDowns[i] = new NumericUpDown
                {
                    Minimum = 0,
                    Maximum = 4095,
                    Value = 0,
                    Location = new Point(x, y + 80),
                    Size = new Size(180, 28),
                    Font = new Font("Consolas", 11),
                    BackColor = Color.White,
                    TextAlign = HorizontalAlignment.Center,
                    DecimalPlaces = 0
                };
                numericUpDowns[idx].ValueChanged += (s, e) => OnNumericChanged(idx, (int)numericUpDowns[idx].Value);

                _rxValueLabels[idx] = new Label
                {
                    Text = "飞控: ---",
                    Location = new Point(x, y + 115),
                    Size = new Size(180, 22),
                    Font = new Font("Consolas", 9, FontStyle.Bold),
                    ForeColor = Color.FromArgb(0, 210, 180),
                    BackColor = Color.FromArgb(55, 55, 60),
                    TextAlign = ContentAlignment.MiddleCenter
                };

                this.Controls.Add(labels[i]);
                this.Controls.Add(trackBars[i]);
                this.Controls.Add(numericUpDowns[i]);
                this.Controls.Add(_rxValueLabels[i]);
            }

            btnSyncAll = new Button
            {
                Text = "同步所有电机",
                Location = new Point(20, 172),
                Size = new Size(180, 35),
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("微软雅黑", 9, FontStyle.Bold)
            };
            btnSyncAll.Click += (s, e) => SetAllThrottle((int)numericUpDowns[0].Value);

            btnEmergencyStop = new Button
            {
                Text = "紧急停止",
                Location = new Point(220, 172),
                Size = new Size(180, 35),
                BackColor = Color.FromArgb(220, 53, 69),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("微软雅黑", 9, FontStyle.Bold)
            };
            btnEmergencyStop.Click += (s, e) => SetAllThrottle(0);

            _rxStatusLabel = new Label
            {
                Text = "等待飞控数据...",
                Location = new Point(420, 177),
                Size = new Size(390, 25),
                Font = new Font("微软雅黑", 9),
                ForeColor = Color.FromArgb(150, 150, 155),
                TextAlign = ContentAlignment.MiddleLeft
            };

            this.Controls.Add(btnSyncAll);
            this.Controls.Add(btnEmergencyStop);
            this.Controls.Add(_rxStatusLabel);
        }

        // ===================== 新增: 姿态/角速度目标 UI =====================
        private void InitializeTargetControls()
        {
            var groupBox = new GroupBox
            {
                Text = "目标姿态角 / 角速度 (msgid=350)",
                Location = new Point(15, 215),
                Size = new Size(840, 210),
                Font = new Font("微软雅黑", 10, FontStyle.Bold),
                ForeColor = Color.White,
                BackColor = Color.FromArgb(50, 50, 55)
            };

            // --- 左侧: 角度目标 (3 个轴) ---
            Color angleColor = Color.FromArgb(0, 180, 220);
            int leftX = 15, rightX = 435;
            int yStart = 20, rowH = 58;

            for (int i = 0; i < 3; i++)
            {
                int idx = i;
                int y = yStart + i * rowH;

                _angleLbls[i] = new Label
                {
                    Text = AngleAxisNames[i] + " 角度",
                    Location = new Point(leftX, y + 10),
                    Size = new Size(85, 25),
                    Font = new Font("微软雅黑", 9, FontStyle.Bold),
                    ForeColor = angleColor,
                    TextAlign = ContentAlignment.MiddleRight
                };

                _angleTbs[i] = new TrackBar
                {
                    Minimum = 0,
                    Maximum = ANGLE_TB_MAX,
                    Value = ANGLE_TB_MAX / 2,
                    Location = new Point(leftX + 90, y + 2),
                    Size = new Size(180, 40),
                    Orientation = Orientation.Horizontal,
                    TickFrequency = 100,
                    LargeChange = 50,
                    SmallChange = 5,
                    BackColor = Color.FromArgb(60, 60, 65),
                    TickStyle = TickStyle.None
                };
                _angleTbs[idx].Scroll += (s, e) => OnAngleTrackBarChanged(idx);

                _angleNuds[i] = new NumericUpDown
                {
                    Minimum = (decimal)(-ANGLE_MAX_RAD),
                    Maximum = (decimal)ANGLE_MAX_RAD,
                    Value = 0,
                    DecimalPlaces = 3,
                    Increment = 0.01m,
                    Location = new Point(leftX + 90 + 185, y + 8),
                    Size = new Size(85, 25),
                    Font = new Font("Consolas", 9),
                    BackColor = Color.White,
                    TextAlign = HorizontalAlignment.Center
                };
                _angleNuds[idx].ValueChanged += (s, e) => OnAngleNudChanged(idx);

                _angleDegLbls[i] = new Label
                {
                    Text = "= 0.0°",
                    Location = new Point(leftX + 90 + 185 + 90, y + 10),
                    Size = new Size(60, 25),
                    Font = new Font("Consolas", 8),
                    ForeColor = Color.FromArgb(140, 200, 220),
                    TextAlign = ContentAlignment.MiddleLeft
                };

                groupBox.Controls.Add(_angleLbls[i]);
                groupBox.Controls.Add(_angleTbs[i]);
                groupBox.Controls.Add(_angleNuds[i]);
                groupBox.Controls.Add(_angleDegLbls[i]);
            }

            // --- 右侧: 角速度目标 (3 个轴) ---
            Color gyroColor = Color.FromArgb(220, 160, 40);

            for (int i = 0; i < 3; i++)
            {
                int idx = i;
                int y = yStart + i * rowH;

                _gyroLbls[i] = new Label
                {
                    Text = "Gyro " + GyroAxisNames[i],
                    Location = new Point(rightX, y + 10),
                    Size = new Size(70, 25),
                    Font = new Font("微软雅黑", 9, FontStyle.Bold),
                    ForeColor = gyroColor,
                    TextAlign = ContentAlignment.MiddleRight
                };

                _gyroTbs[i] = new TrackBar
                {
                    Minimum = 0,
                    Maximum = GYRO_TB_MAX,
                    Value = GYRO_TB_MAX / 2,
                    Location = new Point(rightX + 75, y + 2),
                    Size = new Size(180, 40),
                    Orientation = Orientation.Horizontal,
                    TickFrequency = 100,
                    LargeChange = 50,
                    SmallChange = 5,
                    BackColor = Color.FromArgb(60, 60, 65),
                    TickStyle = TickStyle.None
                };
                _gyroTbs[idx].Scroll += (s, e) => OnGyroTrackBarChanged(idx);

                _gyroNuds[i] = new NumericUpDown
                {
                    Minimum = (decimal)(-GYRO_MAX_RAD),
                    Maximum = (decimal)GYRO_MAX_RAD,
                    Value = 0,
                    DecimalPlaces = 2,
                    Increment = 0.1m,
                    Location = new Point(rightX + 75 + 185, y + 8),
                    Size = new Size(85, 25),
                    Font = new Font("Consolas", 9),
                    BackColor = Color.White,
                    TextAlign = HorizontalAlignment.Center
                };
                _gyroNuds[idx].ValueChanged += (s, e) => OnGyroNudChanged(idx);

                groupBox.Controls.Add(_gyroLbls[i]);
                groupBox.Controls.Add(_gyroTbs[i]);
                groupBox.Controls.Add(_gyroNuds[i]);
            }

            this.Controls.Add(groupBox);
        }

        // ===================== 角度控件事件 =====================
        private void OnAngleTrackBarChanged(int idx)
        {
            if (_targetSuppress) return;
            float rad = TrackBarToAngle(_angleTbs[idx].Value);

            _targetSuppress = true;
            _angleNuds[idx].Value = (decimal)rad;
            _targetSuppress = false;

            UpdateAngleDegLabel(idx, rad);
            TrySendTargets();
        }

        private void OnAngleNudChanged(int idx)
        {
            if (_targetSuppress) return;
            float rad = (float)_angleNuds[idx].Value;

            _targetSuppress = true;
            _angleTbs[idx].Value = AngleToTrackBar(rad);
            _targetSuppress = false;

            UpdateAngleDegLabel(idx, rad);
            TrySendTargets();
        }

        private void UpdateAngleDegLabel(int idx, float rad)
        {
            float deg = rad * 180.0f / (float)Math.PI;
            _angleDegLbls[idx].Text = string.Format("= {0:F1}°", deg);
        }

        // ===================== 角速度控件事件 =====================
        private void OnGyroTrackBarChanged(int idx)
        {
            if (_targetSuppress) return;
            float rads = TrackBarToGyro(_gyroTbs[idx].Value);

            _targetSuppress = true;
            _gyroNuds[idx].Value = (decimal)rads;
            _targetSuppress = false;

            TrySendTargets();
        }

        private void OnGyroNudChanged(int idx)
        {
            if (_targetSuppress) return;
            float rads = (float)_gyroNuds[idx].Value;

            _targetSuppress = true;
            _gyroTbs[idx].Value = GyroToTrackBar(rads);
            _targetSuppress = false;

            TrySendTargets();
        }

        // ===================== TrackBar ↔ 浮点值 映射 =====================
        private static float TrackBarToAngle(int tbVal)
        {
            return (tbVal - ANGLE_TB_MAX / 2) * (ANGLE_MAX_RAD * 2.0f) / ANGLE_TB_MAX;
        }
        private static int AngleToTrackBar(float rad)
        {
            int val = (int)(ANGLE_TB_MAX / 2 + rad * ANGLE_TB_MAX / (ANGLE_MAX_RAD * 2.0f));
            if (val < 0) val = 0;
            if (val > ANGLE_TB_MAX) val = ANGLE_TB_MAX;
            return val;
        }

        private static float TrackBarToGyro(int tbVal)
        {
            return (tbVal - GYRO_TB_MAX / 2) * (GYRO_MAX_RAD * 2.0f) / GYRO_TB_MAX;
        }
        private static int GyroToTrackBar(float rads)
        {
            int val = (int)(GYRO_TB_MAX / 2 + rads * GYRO_TB_MAX / (GYRO_MAX_RAD * 2.0f));
            if (val < 0) val = 0;
            if (val > GYRO_TB_MAX) val = GYRO_TB_MAX;
            return val;
        }

        // ===================== 发送目标值 =====================
        private void TrySendTargets()
        {
            // 频率限制: 50ms 最短间隔
            double elapsed = (DateTime.Now - _lastTargetSend).TotalMilliseconds;
            if (elapsed < TARGET_SEND_INTERVAL_MS)
                return;

            _lastTargetSend = DateTime.Now;

            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                return;

            // 读取当前所有目标值
            float[] data = new float[10];
            for (int i = 0; i < 3; i++)
            {
                data[i]     = (float)_angleNuds[i].Value;
                data[i + 3] = (float)_gyroNuds[i].Value;
            }

            Task.Run(() =>
            {
                try
                {
                    SendDebugFloatArray("ATTI", data);
                }
                catch (Exception ex)
                {
                    Console.WriteLine("[ThrottlePlugin] DEBUG_FLOAT_ARRAY send error: " + ex.Message);
                }
            });
        }

        // 诊断: 已发送计数
        private static int _sendCount = 0;

        private void SendDebugFloatArray(string name, float[] data)
        {
            var comPort = MainV2.comPort;
            byte sysid, compid;
            try { sysid = comPort.MAV.sysid; compid = comPort.MAV.compid; }
            catch { sysid = 255; compid = 1; }

            try
            {
                // 构造 DEBUG_FLOAT_ARRAY 负载 (60 bytes)
                byte[] payload = new byte[60];

                // time_usec (uint64_t, offset 0)
                ulong timeUsec = (ulong)(DateTime.UtcNow.Ticks / 10);
                Buffer.BlockCopy(BitConverter.GetBytes(timeUsec), 0, payload, 0, 8);

                // name (char[10], offset 8)
                byte[] nameBytes = new byte[10];
                byte[] nameRaw = System.Text.Encoding.ASCII.GetBytes(name);
                int copyLen = Math.Min(nameRaw.Length, 10);
                Array.Copy(nameRaw, 0, nameBytes, 0, copyLen);
                Array.Copy(nameBytes, 0, payload, 8, 10);

                // array_id (uint16_t, offset 18)
                Buffer.BlockCopy(BitConverter.GetBytes((ushort)0), 0, payload, 18, 2);

                // data (float[10], offset 20)
                for (int i = 0; i < 10; i++)
                {
                    float val = (i < data.Length) ? data[i] : 0.0f;
                    Buffer.BlockCopy(BitConverter.GetBytes(val), 0, payload, 20 + i * 4, 4);
                }

                // 用自建的 MAVLink v2 封包 (不依赖 MP 内置库, 确保 CRC_EXTRA=232 正确)
                byte[] fullPacket = BuildMavLink2Packet(payload, 60, sysid, compid, 
                    MSG_ID_DEBUG_FLOAT_ARRAY, CRC_EXTRA_DEBUG_FLOAT_ARRAY);
                comPort.BaseStream.Write(fullPacket, 0, fullPacket.Length);
                _sendCount++;
                if (_sendCount % 10 == 0)
                    Console.WriteLine("[ThrottlePlugin] sent OK, total=" + _sendCount);
            }
            catch (Exception ex)
            {
                Console.WriteLine("[ThrottlePlugin] SendDebugFloatArray failed: " + ex.Message);
            }
        }

        // CRC_EXTRA for DEBUG_FLOAT_ARRAY (msgid=350)
        private const byte CRC_EXTRA_DEBUG_FLOAT_ARRAY = 232;

        // 构建正确 MAVLink v2 数据包
        // v2: STX(0xFD) LEN FLAGS(2) SEQ SYS COMP MSGID(3) PAYLOAD CRC(2)
        private static byte[] BuildMavLink2Packet(byte[] payload, int payloadLen, byte sysid, byte compid, int msgid, byte crcExtra)
        {
            int packetLen = 10 + payloadLen + 2;  // header(10) + payload + CRC(2)
            byte[] packet = new byte[packetLen];
            int pos = 0;

            // MAVLink v2 帧头
            packet[pos++] = 0xFD;                             // STX (v2 魔数)
            packet[pos++] = (byte)payloadLen;                 // LEN
            packet[pos++] = 0;                                // incompatibility_flags
            packet[pos++] = 0;                                // compatibility_flags
            packet[pos++] = 0;                                // SEQ (占位)
            packet[pos++] = sysid;                            // SYSID
            packet[pos++] = compid;                           // COMPID
            packet[pos++] = (byte)(msgid & 0xFF);             // MSGID[0]
            packet[pos++] = (byte)((msgid >> 8) & 0xFF);      // MSGID[1]
            packet[pos++] = (byte)((msgid >> 16) & 0xFF);     // MSGID[2]

            // 负载
            Array.Copy(payload, 0, packet, pos, payloadLen);
            pos += payloadLen;

            // CRC-16/MCRF4XX: 先处理 LEN 到 payload 末尾的所有字节，最后才累加 CRC_EXTRA
            ushort crc = 0xFFFF;  // X25_INIT_CRC
            for (int i = 1; i < pos; i++)
                crc = Crc16Accumulate(packet[i], crc);
            crc = Crc16Accumulate(crcExtra, crc);  // CRC_EXTRA 必须最后累加!

            packet[pos++] = (byte)(crc & 0xFF);
            packet[pos++] = (byte)((crc >> 8) & 0xFF);

            return packet;
        }

        // MAVLink CRC-16/X.25 单字节累计
        private static ushort Crc16Accumulate(byte data, ushort crc)
        {
            byte tmp = (byte)(data ^ (crc & 0xFF));
            tmp ^= (byte)(tmp << 4);
            return (ushort)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
        }

        // ===================== 原有: 电机油门事件 =====================
        private void OnThrottleChanged(int channel, int value)
        {
            if (_suppressEvents) return;
            numericUpDowns[channel].Value = value;
            QueueCommand(channel, value);
        }

        private void OnNumericChanged(int channel, int value)
        {
            if (_suppressEvents) return;
            if (trackBars[channel].Value != value)
                trackBars[channel].Value = value;
            else
                QueueCommand(channel, value);
        }

        private void SetAllThrottle(int value)
        {
            _suppressEvents = true;
            for (int i = 0; i < 4; i++)
            {
                trackBars[i].Value = value;
                numericUpDowns[i].Value = value;
                _lastSendTime[i] = DateTime.Now;
            }
            _suppressEvents = false;

            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                return;

            var comPort = MainV2.comPort;
            int val = value;
            Task.Run(() =>
            {
                for (int i = 0; i < 4; i++)
                {
                    try
                    {
                        comPort.doCommand(
                            MAVLink.MAV_CMD.DO_SET_SERVO,
                            (float)(9 + i),
                            (float)val,
                            0, 0, 0, 0, 0,
                            false);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine(string.Format("Error ch{0}: {1}", i, ex.Message));
                    }
                    Thread.Sleep(15);
                }
            });
        }

        private void QueueCommand(int channel, int value)
        {
            _pendingValues[channel] = value;
            _pendingChannels[channel] = true;
            _debounceTimer.Stop();
            _debounceTimer.Start();
        }

        private void OnDebounceTick(object sender, EventArgs e)
        {
            _debounceTimer.Stop();
            for (int i = 0; i < 4; i++)
            {
                if (_pendingChannels[i])
                {
                    _pendingChannels[i] = false;
                    SendThrottleNow(i, _pendingValues[i]);
                }
            }
        }

        private void SendThrottleNow(int channel, int value)
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
                return;

            TimeSpan elapsed = DateTime.Now - _lastSendTime[channel];
            if (elapsed.TotalMilliseconds < 50)
                return;

            _lastSendTime[channel] = DateTime.Now;
            DoSend(channel, value);
        }

        private void DoSend(int channel, int value)
        {
            var comPort = MainV2.comPort;
            int ch = channel;
            int val = value;
            Task.Run(() =>
            {
                try
                {
                    comPort.doCommand(
                        MAVLink.MAV_CMD.DO_SET_SERVO,
                        (float)(9 + ch),
                        (float)val,
                        0, 0, 0, 0, 0,
                        false);
                }
                catch (Exception ex)
                {
                    Console.WriteLine(string.Format("Error: {0}", ex.Message));
                }
            });
        }

        // ===================== MAVLink 订阅 =====================
        private void SubscribeMavlink()
        {
            try
            {
                if (MainV2.comPort != null)
                    MainV2.comPort.OnPacketReceived += OnMavlinkPacket;
            }
            catch { }

            var retryTimer = new System.Windows.Forms.Timer { Interval = 2000 };
            retryTimer.Tick += (s, e) =>
            {
                try
                {
                    if (MainV2.comPort != null)
                    {
                        MainV2.comPort.OnPacketReceived -= OnMavlinkPacket;
                        MainV2.comPort.OnPacketReceived += OnMavlinkPacket;
                        retryTimer.Stop();
                        retryTimer.Dispose();
                    }
                }
                catch { }
            };
            retryTimer.Start();
        }

        private void OnMavlinkPacket(object sender, MAVLink.MAVLinkMessage msg)
        {
            _totalPacketCount++;

            lock (_msgIdCounts)
            {
                if (!_msgIdCounts.ContainsKey(msg.msgid))
                    _msgIdCounts[msg.msgid] = 0;
                _msgIdCounts[msg.msgid]++;
            }

            // --- SERVO_OUTPUT_RAW (msgid=36) ---
            if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW)
            {
                _servoPacketCount++;
                try
                {
                    var servo = (MAVLink.mavlink_servo_output_raw_t)msg.data;
                    lock (_rxLock)
                    {
                        _receivedServo[0] = servo.servo1_raw;
                        _receivedServo[1] = servo.servo2_raw;
                        _receivedServo[2] = servo.servo3_raw;
                        _receivedServo[3] = servo.servo4_raw;
                    }
                    _hasServoData = true;
                    _msgSource = "SERVO_OUTPUT_RAW(1-4)";
                }
                catch
                {
                    try
                    {
                        var servo = (MAVLink.mavlink_servo_output_raw_t)msg.data;
                        lock (_rxLock)
                        {
                            _receivedServo[0] = servo.servo9_raw;
                            _receivedServo[1] = servo.servo10_raw;
                            _receivedServo[2] = servo.servo11_raw;
                            _receivedServo[3] = servo.servo12_raw;
                        }
                        _hasServoData = true;
                        _msgSource = "SERVO_OUTPUT_RAW(9-12)";
                    }
                    catch { }
                }
                return;
            }

            if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.RC_CHANNELS)
            {
                _rcPacketCount++;
                try
                {
                    var rc = (MAVLink.mavlink_rc_channels_t)msg.data;
                    lock (_rxLock)
                    {
                        _receivedServo[0] = rc.chan1_raw;
                        _receivedServo[1] = rc.chan2_raw;
                        _receivedServo[2] = rc.chan3_raw;
                        _receivedServo[3] = rc.chan4_raw;
                    }
                    _hasServoData = true;
                    _msgSource = "RC_CHANNELS(65)";
                }
                catch { }
                return;
            }

            if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW)
            {
                _rcPacketCount++;
                try
                {
                    var rc = (MAVLink.mavlink_rc_channels_raw_t)msg.data;
                    lock (_rxLock)
                    {
                        _receivedServo[0] = rc.chan1_raw;
                        _receivedServo[1] = rc.chan2_raw;
                        _receivedServo[2] = rc.chan3_raw;
                        _receivedServo[3] = rc.chan4_raw;
                    }
                    _hasServoData = true;
                    _msgSource = "RC_CHANNELS_RAW";
                }
                catch { }
            }
        }

        private void RequestServoStream(object sender, EventArgs e)
        {
            if (MainV2.comPort == null || MainV2.comPort.BaseStream == null || !MainV2.comPort.BaseStream.IsOpen)
                return;

            byte sysid = 1, compid = 1;
            try { sysid = MainV2.comPort.MAV.sysid; compid = MainV2.comPort.MAV.compid; }
            catch { }

            try
            {
                MainV2.comPort.requestDatastream(MAVLink.MAV_DATA_STREAM.RC_CHANNELS, 10, sysid, compid);
            }
            catch { }

            try
            {
                MainV2.comPort.doCommand(
                    MAVLink.MAV_CMD.SET_MESSAGE_INTERVAL,
                    (float)(uint)MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW,
                    100000, 0, 0, 0, 0, 0, false);
            }
            catch { }

            try
            {
                MainV2.comPort.doCommand(
                    MAVLink.MAV_CMD.SET_MESSAGE_INTERVAL,
                    (float)(uint)MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW,
                    100000, 0, 0, 0, 0, 0, false);
            }
            catch { }
        }

        private void OnRxRefresh(object sender, EventArgs e)
        {
            float[] current;
            lock (_rxLock)
            {
                current = (float[])_receivedServo.Clone();
            }

            bool hasData = _hasServoData && (current[0] > 0 || current[1] > 0 || current[2] > 0 || current[3] > 0);

            for (int i = 0; i < 4; i++)
            {
                if (_rxValueLabels[i] == null) continue;
                if (hasData)
                    _rxValueLabels[i].Text = string.Format("飞控: {0}", (int)current[i]);
                else
                    _rxValueLabels[i].Text = "飞控: ---";
            }

            if (_rxStatusLabel != null)
            {
                if (hasData)
                    _rxStatusLabel.Text = string.Format("数据正常 [{0}] | 总包:{1} 伺服:{2} RC:{3}",
                        _msgSource, _totalPacketCount, _servoPacketCount, _rcPacketCount);
                else if (_totalPacketCount > 0 && _servoPacketCount == 0 && _rcPacketCount == 0)
                {
                    string diagInfo = "";
                    lock (_msgIdCounts)
                    {
                        var topIds = _msgIdCounts
                            .OrderByDescending(kv => kv.Value)
                            .Take(5)
                            .Select(kv => string.Format("#{0}x{1}", kv.Key, kv.Value));
                        diagInfo = string.Join(" ", topIds);
                    }
                    _rxStatusLabel.Text = string.Format("已收到 {0} 包 无伺服/RC | 消息ID: {1} | 尝试请求中...",
                        _totalPacketCount, string.IsNullOrEmpty(diagInfo) ? "-" : diagInfo);
                }
                else if (_totalPacketCount > 0 && (_servoPacketCount > 0 || _rcPacketCount > 0))
                    _rxStatusLabel.Text = string.Format("收到伺服:{0} RC:{1} 但数值为0 | 检查通道", _servoPacketCount, _rcPacketCount);
                else if (_totalPacketCount == 0)
                    _rxStatusLabel.Text = "等待飞控数据... (确认飞控已连接)";
                else
                    _rxStatusLabel.Text = "等待飞控数据...";
            }
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            if (_rxRefreshTimer != null) { _rxRefreshTimer.Stop(); _rxRefreshTimer.Dispose(); }
            if (_streamReqTimer != null) { _streamReqTimer.Stop(); _streamReqTimer.Dispose(); }
            try { MainV2.comPort.OnPacketReceived -= OnMavlinkPacket; } catch { }
            base.OnFormClosing(e);
        }
    }
}
