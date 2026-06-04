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
        private int _statustextPacketCount = 0;    // 诊断: 收到msgid=253的次数
        private int _nvfPacketCount = 0;            // 诊断: 收到msgid=251的次数
        private int _sysStatusPacketCount = 0;      // 诊断: 收到msgid=1 (SYS_STATUS) 的次数

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

        // ===================== PID 参数控件 =====================
        private TrackBar _tbT;
        private NumericUpDown _nudT;
        private Label _lblTVal;

        private TrackBar[] _pidTbs = new TrackBar[9];   // [0-2]Roll Kp/Ki/Kd, [3-5]Pitch, [6-8]Yaw
        private NumericUpDown[] _pidNuds = new NumericUpDown[9];
        private bool _pidSuppress;
        private DateTime _lastPidSend = DateTime.MinValue;
        private Button _btnSendPid, _btnLoadDefaults, _btnQueryPid;
        private Label _lblPidStatus;
        private const int PID_SEND_INTERVAL_MS = 100;

        private const int PID_TB_MAX = 3000;       // Kp/Ki/Kd: 0.0~300.0, step 0.1
        private const int PID_T_MAX = 100;          // T: 0.00~1.00, step 0.01

        private static readonly string[] PidAxisNames = { "Roll", "Pitch", "Yaw" };
        private static readonly string[] PidCompNames = { "Kp", "Ki", "Kd" };

        private const int MSG_ID_DEBUG_FLOAT_ARRAY = 350;    // 发送用
        private const int MSG_ID_NAMED_VALUE_FLOAT = 251;    // 接收 PIDR 用

        // ===================== 电池电压显示 =====================
        private float _busVoltage = 0f;             // 母线电压 (V)
        private float _cellVoltage = 0f;            // 单节电压 (V)
        private int   _batteryRemaining = -1;       // 剩余电量 % (-1=未知)
        private DateTime _lastBatteryTime = DateTime.MinValue;
        private bool _batteryLow = false;
        private Label _lblBattery;                  // 电池信息显示标签
        private const int BATTERY_CELL_COUNT = 6;   // 6S 电池
        private const float CELL_LOW_THRESHOLD = 3.65f;  // 单节低电压阈值
        private const double BATTERY_STALE_SEC = 5.0;     // 超时判定 (秒)

        public ThrottleControlForm()
        {
            InitializeBatteryPanel();
            InitializeComponents();
            InitializeTargetControls();
            InitializePidControls();
            this.Height = 780;

            _debounceTimer = new System.Windows.Forms.Timer { Interval = 50 };
            _debounceTimer.Tick += OnDebounceTick;
            SubscribeMavlink();
            _rxRefreshTimer = new System.Windows.Forms.Timer { Interval = 200 };
            _rxRefreshTimer.Tick += OnRxRefresh;
            _rxRefreshTimer.Start();

            _streamReqTimer = new System.Windows.Forms.Timer { Interval = 3000 };
            _streamReqTimer.Tick += RequestServoStream;
            _streamReqTimer.Start();

            // 电池电压刷新 / 超时检测定时器 (2秒)
            var batteryTimer = new System.Windows.Forms.Timer { Interval = 2000 };
            batteryTimer.Tick += (s, e) => { this.BeginInvoke((Action)(() => UpdateBatteryDisplay())); };
            batteryTimer.Start();
        }

        // ===================== 电池电压 UI =====================
        private void InitializeBatteryPanel()
        {
            _lblBattery = new Label
            {
                Text = "电池: 等待数据...",
                Location = new Point(10, 5),
                Size = new Size(840, 30),
                Font = new Font("Consolas", 11, FontStyle.Bold),
                ForeColor = Color.FromArgb(0, 255, 170),
                BackColor = Color.FromArgb(55, 55, 60),
                TextAlign = ContentAlignment.MiddleCenter
            };
            this.Controls.Add(_lblBattery);
        }

        private void UpdateBatteryDisplay()
        {
            if (_lblBattery == null) return;

            double elapsed = (DateTime.Now - _lastBatteryTime).TotalSeconds;
            bool stale = _lastBatteryTime == DateTime.MinValue || elapsed > BATTERY_STALE_SEC;

            if (stale)
            {
                _lblBattery.Text = "电池: 数据超时 / 等待中...";
                _lblBattery.ForeColor = Color.FromArgb(150, 150, 155);
                _lblBattery.BackColor = Color.FromArgb(55, 55, 60);
                return;
            }

            string battPct = _batteryRemaining >= 0 ? $"{_batteryRemaining}%" : "--";
            string cellStr = _cellVoltage > 0 ? $"{_cellVoltage:F2} V" : "--";
            string busStr  = _busVoltage > 0 ? $"{_busVoltage:F2} V" : "--";

            _lblBattery.Text = $"母线: {busStr}  │  单节: {cellStr}  │  电量: {battPct}";

            if (_batteryLow)
            {
                _lblBattery.ForeColor = Color.White;
                _lblBattery.BackColor = Color.FromArgb(220, 50, 50);   // 醒目红色预警
            }
            else
            {
                _lblBattery.ForeColor = Color.FromArgb(0, 255, 170);
                _lblBattery.BackColor = Color.FromArgb(55, 55, 60);
            }
        }

        // ===================== 原有: 电机油门 UI =====================
        private void InitializeComponents()
        {
            this.Text = "飞控综合调参面板";
            this.Size = new Size(870, 740);
            this.StartPosition = FormStartPosition.CenterScreen;
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.BackColor = Color.FromArgb(45, 45, 48);

            for (int i = 0; i < 4; i++)
            {
                int idx = i;
                int x = 20 + i * 200;
                int y = 65;

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
                Location = new Point(20, 207),
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
                Location = new Point(220, 207),
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
                Location = new Point(420, 212),
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
                Location = new Point(15, 250),
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

        // ===================== PID 参数调谐 UI =====================
        private void InitializePidControls()
        {
            var gb = new GroupBox
            {
                Text = "PID 参数调谐 (msgid=350, name=PIDP)",
                Location = new Point(15, 470),
                Size = new Size(840, 265),
                Font = new Font("微软雅黑", 10, FontStyle.Bold),
                ForeColor = Color.White,
                BackColor = Color.FromArgb(50, 50, 55)
            };

            // --- T (时间常数) + 按钮行 ---
            int row0y = 25;
            var lblT = new Label
            {
                Text = "T(s):",
                Location = new Point(20, row0y + 5),
                Size = new Size(40, 25),
                Font = new Font("微软雅黑", 9, FontStyle.Bold),
                ForeColor = Color.FromArgb(0, 255, 170),
                TextAlign = ContentAlignment.MiddleRight
            };

            _tbT = new TrackBar
            {
                Minimum = 0, Maximum = PID_T_MAX, Value = 50,  // default 0.50
                Location = new Point(65, row0y),
                Size = new Size(140, 40),
                TickFrequency = 10, LargeChange = 10, SmallChange = 1,
                BackColor = Color.FromArgb(60, 60, 65),
                TickStyle = TickStyle.None
            };
            _tbT.Scroll += (s, e) => OnPidTChanged();

            _nudT = new NumericUpDown
            {
                Minimum = 0.00m, Maximum = 1.00m, Value = 0.50m,
                DecimalPlaces = 2, Increment = 0.01m,
                Location = new Point(210, row0y + 3),
                Size = new Size(65, 25),
                Font = new Font("Consolas", 9),
                BackColor = Color.White, TextAlign = HorizontalAlignment.Center
            };
            _nudT.ValueChanged += (s, e) => OnPidTNudChanged();

            _lblTVal = new Label
            {
                Text = "s",
                Location = new Point(280, row0y + 5),
                Size = new Size(25, 25),
                Font = new Font("Consolas", 9),
                ForeColor = Color.FromArgb(180, 180, 185),
                TextAlign = ContentAlignment.MiddleLeft
            };

            _btnSendPid = new Button
            {
                Text = "发送 PID",
                Location = new Point(330, row0y + 2),
                Size = new Size(85, 30),
                BackColor = Color.FromArgb(0, 122, 204),
                ForeColor = Color.White, FlatStyle = FlatStyle.Flat,
                Font = new Font("微软雅黑", 9, FontStyle.Bold)
            };
            _btnSendPid.Click += (s, e) => SendPidParams();

            _btnLoadDefaults = new Button
            {
                Text = "推荐值",
                Location = new Point(425, row0y + 2),
                Size = new Size(75, 30),
                BackColor = Color.FromArgb(80, 80, 90),
                ForeColor = Color.White, FlatStyle = FlatStyle.Flat,
                Font = new Font("微软雅黑", 9, FontStyle.Bold)
            };
            _btnLoadDefaults.Click += (s, e) => LoadPidDefaults();

            _btnQueryPid = new Button
            {
                Text = "查询 PID",
                Location = new Point(510, row0y + 2),
                Size = new Size(85, 30),
                BackColor = Color.FromArgb(0, 150, 100),
                ForeColor = Color.White, FlatStyle = FlatStyle.Flat,
                Font = new Font("微软雅黑", 9, FontStyle.Bold)
            };
            _btnQueryPid.Click += (s, e) => QueryPidFromFc();

            _lblPidStatus = new Label
            {
                Text = "",
                Location = new Point(15, row0y + 35),
                Size = new Size(580, 22),
                Font = new Font("微软雅黑", 9),
                ForeColor = Color.FromArgb(0, 200, 160),
                TextAlign = ContentAlignment.MiddleLeft
            };

            gb.Controls.Add(lblT);
            gb.Controls.Add(_tbT);
            gb.Controls.Add(_nudT);
            gb.Controls.Add(_lblTVal);
            gb.Controls.Add(_btnSendPid);
            gb.Controls.Add(_btnLoadDefaults);
            gb.Controls.Add(_btnQueryPid);
            gb.Controls.Add(_lblPidStatus);

            // --- 三轴 PID 行 ---
            int[] colX = { 70, 300, 530 };  // Kp / Ki / Kd 列起始 X
            Color[] axisColors = {
                Color.FromArgb(255, 100, 100),  // Roll - 红
                Color.FromArgb(100, 220, 100),  // Pitch - 绿
                Color.FromArgb(100, 150, 255)   // Yaw - 蓝
            };

            for (int axis = 0; axis < 3; axis++)
            {
                int rowY = 75 + axis * 55;
                int baseIdx = axis * 3;

                // 轴名称标签
                var lblAxis = new Label
                {
                    Text = PidAxisNames[axis],
                    Location = new Point(15, rowY + 8),
                    Size = new Size(50, 25),
                    Font = new Font("微软雅黑", 9, FontStyle.Bold),
                    ForeColor = axisColors[axis],
                    TextAlign = ContentAlignment.MiddleRight
                };
                gb.Controls.Add(lblAxis);

                for (int comp = 0; comp < 3; comp++)
                {
                    int idx = baseIdx + comp;
                    int cx = colX[comp];

                    // 分量标签 (Kp / Ki / Kd)
                    var lblComp = new Label
                    {
                        Text = PidCompNames[comp],
                        Location = new Point(cx, rowY + 8),
                        Size = new Size(25, 25),
                        Font = new Font("Consolas", 9, FontStyle.Bold),
                        ForeColor = Color.FromArgb(200, 200, 210),
                        TextAlign = ContentAlignment.MiddleRight
                    };

                    _pidTbs[idx] = new TrackBar
                    {
                        Minimum = 0, Maximum = PID_TB_MAX, Value = 0,
                        Location = new Point(cx + 28, rowY + 2),
                        Size = new Size(115, 40),
                        TickFrequency = 200, LargeChange = 50, SmallChange = 5,
                        BackColor = Color.FromArgb(60, 60, 65),
                        TickStyle = TickStyle.None
                    };
                    int captureIdx = idx;
                    _pidTbs[idx].Scroll += (s, e) => OnPidTrackBarChanged(captureIdx);

                    _pidNuds[idx] = new NumericUpDown
                    {
                        Minimum = 0.0m, Maximum = 300.0m, Value = 0.0m,
                        DecimalPlaces = 1, Increment = 0.1m,
                        Location = new Point(cx + 145, rowY + 5),
                        Size = new Size(65, 25),
                        Font = new Font("Consolas", 9),
                        BackColor = Color.White, TextAlign = HorizontalAlignment.Center
                    };
                    _pidNuds[idx].ValueChanged += (s, e) => OnPidNudChanged(captureIdx);

                    gb.Controls.Add(lblComp);
                    gb.Controls.Add(_pidTbs[idx]);
                    gb.Controls.Add(_pidNuds[idx]);
                }
            }

            this.Controls.Add(gb);
        }

        // ===================== PID T (时间常数) 事件 =====================
        private void OnPidTChanged()
        {
            if (_pidSuppress) return;
            float val = _tbT.Value / 100.0f;
            _pidSuppress = true;
            _nudT.Value = (decimal)val;
            _pidSuppress = false;
            // 不自动发送, 需点「发送 PID」按钮
        }

        private void OnPidTNudChanged()
        {
            if (_pidSuppress) return;
            int tb = (int)(_nudT.Value * 100m);
            if (tb < 0) tb = 0; if (tb > PID_T_MAX) tb = PID_T_MAX;
            _pidSuppress = true;
            _tbT.Value = tb;
            _pidSuppress = false;
            // 不自动发送, 需点「发送 PID」按钮
        }

        // ===================== PID Kp/Ki/Kd  事件 =====================
        private void OnPidTrackBarChanged(int idx)
        {
            if (_pidSuppress) return;
            float val = _pidTbs[idx].Value / 10.0f;
            _pidSuppress = true;
            _pidNuds[idx].Value = (decimal)val;
            _pidSuppress = false;
            // 不自动发送, 需点「发送 PID」按钮
        }

        private void OnPidNudChanged(int idx)
        {
            if (_pidSuppress) return;
            int tb = (int)(_pidNuds[idx].Value * 10m);
            if (tb < 0) tb = 0; if (tb > PID_TB_MAX) tb = PID_TB_MAX;
            _pidSuppress = true;
            _pidTbs[idx].Value = tb;
            _pidSuppress = false;
            // 不自动发送, 需点「发送 PID」按钮
        }

        // ===================== 发送 PID 参数 =====================
        private void TrySendPidParams()
        {
            double elapsed = (DateTime.Now - _lastPidSend).TotalMilliseconds;
            if (elapsed < PID_SEND_INTERVAL_MS)
                return;
            _lastPidSend = DateTime.Now;
            SendPidParams();
        }

        private void SendPidParams()
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
            {
                _lblPidStatus.Text = "未连接飞控";
                return;
            }

            // data[0]=T, data[1-3]=Roll Kp/Ki/Kd, data[4-6]=Pitch Kp/Ki/Kd, data[7-9]=Yaw Kp/Ki/Kd
            float[] data = new float[10];
            data[0] = (float)_nudT.Value;
            for (int i = 0; i < 9; i++)
                data[i + 1] = (float)_pidNuds[i].Value;

            int cnt = Interlocked.Increment(ref _pidSendCnt);
            _lastPidSend = DateTime.Now;

            Task.Run(() =>
            {
                try
                {
                    SendDebugFloatArray("PIDP", data);
                    this.BeginInvoke((Action)(() =>
                    {
                        _lblPidStatus.Text = string.Format("已发送 #{0}", cnt);
                    }));
                }
                catch (Exception ex)
                {
                    Console.WriteLine("[PID] send error: " + ex.Message);
                }
            });
        }
        private static int _pidSendCnt = 0;
        private float[] _pidrAccum = new float[10];
        private int _pidrRecvMask = 0;       // 位掩码: bit0~bit9 对应 PIDR0~PIDR9
        private object _pidrLock = new object();
        private int _pidrStaFlags = 0;       // bit0=收到PIDRA, bit1=收到PIDRB

        // ===================== 加载推荐 PID 值 =====================
        private void LoadPidDefaults()
        {
            // 推荐值 (来自 drone_control.c 注释):
            //   Roll/Pitch: Kp=20, Ki=200, Kd=0
            //   Yaw:        Kp=25, Ki=250, Kd=0
            //   T = 0.5
            int[] defaults = {
                20, 200, 0,    // Roll
                20, 200, 0,    // Pitch
                25, 250, 0     // Yaw
            };

            _pidSuppress = true;
            _nudT.Value = 0.50m;
            _tbT.Value = 50;
            for (int i = 0; i < 9; i++)
            {
                _pidNuds[i].Value = defaults[i];
                _pidTbs[i].Value = defaults[i] * 10;
            }
            _pidSuppress = false;

            _lblPidStatus.Text = "已加载推荐值, 点击「发送 PID」";
        }

        // ===================== 查询飞控当前 PID =====================
        private void QueryPidFromFc()
        {
            if (MainV2.comPort == null || !MainV2.comPort.BaseStream.IsOpen)
            {
                _lblPidStatus.Text = "未连接飞控";
                return;
            }

            Task.Run(() =>
            {
                try
                {
                    // 发送空数据的 PIDQ 查询请求
                    SendDebugFloatArray("PIDQ", new float[10]);
                    this.BeginInvoke((Action)(() =>
                    {
                        _lblPidStatus.Text = "已发送查询, 等待飞控回传...";
                    }));
                }
                catch (Exception ex)
                {
                    Console.WriteLine("[PID] query error: " + ex.Message);
                }
            });
        }

        // ===================== 接收 PIDR 回传 =====================
        private void HandlePidResponse(float[] data)
        {
            if (data == null || data.Length < 10) return;

            this.BeginInvoke((Action)(() =>
            {
                _pidSuppress = true;
                _nudT.Value = (decimal)data[0];
                _tbT.Value = (int)(data[0] * 100f);
                for (int i = 0; i < 9; i++)
                {
                    _pidNuds[i].Value = (decimal)data[i + 1];
                    _pidTbs[i].Value = (int)(data[i + 1] * 10f);
                }
                _pidSuppress = false;
                _lblPidStatus.Text = string.Format("已同步飞控 PID | T={0:F2}s Roll:{1:F1}/{2:F1}/{3:F1}",
                    data[0], data[1], data[2], data[3]);
                // 按钮变绿反馈
                if (_btnQueryPid != null)
                {
                    _btnQueryPid.BackColor = Color.FromArgb(0, 200, 80);
                    _btnQueryPid.Text = "✓ 已同步";
                }
            }));
        }

        private static int CountBits(int mask)
        {
            int count = 0;
            while (mask != 0) { count++; mask &= mask - 1; }
            return count;
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
            // 诊断: 统计关键消息类型
            if (msg.msgid == 253L) Interlocked.Increment(ref _statustextPacketCount);
            if (msg.msgid == (uint)MSG_ID_NAMED_VALUE_FLOAT) Interlocked.Increment(ref _nvfPacketCount);
            if (msg.msgid == 1L) Interlocked.Increment(ref _sysStatusPacketCount);

            // --- SYS_STATUS (msgid=1) 电池电压 ---
            if (msg.msgid == 1L)  // MAVLINK_MSG_ID_SYS_STATUS
            {
                try
                {
                    var sys = (MAVLink.mavlink_sys_status_t)msg.data;
                    float voltageV = sys.voltage_battery / 1000.0f;
                    int remaining = sys.battery_remaining;

                    _busVoltage = voltageV;
                    _cellVoltage = voltageV / BATTERY_CELL_COUNT;
                    _batteryRemaining = remaining;
                    _lastBatteryTime = DateTime.Now;
                    _batteryLow = (_cellVoltage > 0 && _cellVoltage < CELL_LOW_THRESHOLD);

                    this.BeginInvoke((Action)(() => UpdateBatteryDisplay()));
                }
                catch { }
                return;
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

            // --- PID 回传通道: STATUSTEXT msgid=253 (双段 PIDRA + PIDRB, 各5值) ---
            if (msg.msgid == 253L)  // MAVLINK_MSG_ID_STATUSTEXT
            {
                try
                {
                    string text = null;
                    if (msg.data is byte[] raw && raw.Length >= 51)
                        text = System.Text.Encoding.ASCII.GetString(raw, 1, 50).TrimEnd('\0');
                    else
                    {
                        try
                        {
                            var st = (MAVLink.mavlink_statustext_t)msg.data;
                            text = System.Text.Encoding.ASCII.GetString(st.text).TrimEnd('\0');
                        }
                        catch
                        {
                            // fallback: raw byte extraction
                            byte[] rb = msg.data as byte[];
                            if (rb == null)
                            {
                                var df = msg.data.GetType().GetField("text");
                                if (df != null)
                                {
                                    var tv = df.GetValue(msg.data);
                                    if (tv is byte[] tb) text = System.Text.Encoding.ASCII.GetString(tb).TrimEnd('\0');
                                    else if (tv is char[] tc) text = new string(tc).TrimEnd('\0');
                                }
                            }
                        }
                    }

                    if (text == null) return;

                    // 双段格式: PIDRA|T|KpR|KiR|KdR|KpP   PIDRB|KiP|KdP|KpY|KiY|KdY
                    bool isA = text.StartsWith("PIDRA|");
                    bool isB = text.StartsWith("PIDRB|");

                    if (isA || isB)
                    {
                        string[] parts = text.Split('|');
                        int startIdx = isA ? 0 : 5;  // PIDRA→indices 0-4, PIDRB→indices 5-9

                        if (parts.Length >= 6)  // prefix + 5 values
                        {
                            bool allOk = true;
                            for (int i = 0; i < 5; i++)
                            {
                                if (!float.TryParse(parts[i + 1],
                                    System.Globalization.NumberStyles.Float,
                                    System.Globalization.CultureInfo.InvariantCulture,
                                    out float v))
                                { allOk = false; break; }
                                _pidrAccum[startIdx + i] = v;
                            }
                            if (allOk)
                            {
                                lock (_pidrLock)
                                {
                                    if (isA) _pidrStaFlags |= 1;
                                    if (isB) _pidrStaFlags |= 2;
                                }
                                this.BeginInvoke((Action)(() =>
                                {
                                    _lblPidStatus.Text = string.Format("STA {0}/{1}",
                                        isA ? "A" : "B",
                                        (_pidrStaFlags == 3) ? "完成" : "等待另一半");
                                }));
                            }
                        }
                    }
                    // 兼容旧版单段截断格式 PIDR|... (最少10段含截断)
                    else if (text.StartsWith("PIDR|"))
                    {
                        string[] parts = text.Split('|');
                        if (parts.Length >= 10)  // 截断后最少有 PIDR + 9 个值
                        {
                            float[] vals = new float[10];
                            bool allOk = true;
                            for (int i = 0; i < parts.Length - 1 && i < 10; i++)
                                allOk &= float.TryParse(parts[i + 1],
                                    System.Globalization.NumberStyles.Float,
                                    System.Globalization.CultureInfo.InvariantCulture, out vals[i]);
                            // 截断缺失的值用0填充
                            for (int i = parts.Length - 1; i < 10; i++)
                                vals[i] = 0f;
                            if (allOk)
                            {
                                HandlePidResponse(vals);
                                this.BeginInvoke((Action)(() =>
                                {
                                    _lblPidStatus.Text = "PIDR via STATUSTEXT(截断) 完成 ✓";
                                }));
                            }
                        }
                    }

                    // 收到双段后组装
                    lock (_pidrLock)
                    {
                        if (_pidrStaFlags == 3)  // A+B 都收到了
                        {
                            float[] data = new float[10];
                            Array.Copy(_pidrAccum, data, 10);
                            _pidrStaFlags = 0;  // 重置
                            HandlePidResponse(data);
                            this.BeginInvoke((Action)(() =>
                            {
                                _lblPidStatus.Text = "PIDR via STATUSTEXT 完成 ✓";
                            }));
                        }
                    }
                }
                catch { }
            }

            // --- PID 回传 (NAMED_VALUE_FLOAT msgid=251, name=PIDR0..PIDR9) ---
            if (msg.msgid == MSG_ID_NAMED_VALUE_FLOAT)
            {
                Type dt = msg.data?.GetType();
                string diagType = dt?.FullName ?? "null";
                string parseName = null;
                float parseValue = 0f;

                try
                {
                    // 方式1: msg.data 是 byte[] (MP 不定制该结构体时)
                    if (msg.data is byte[] raw && raw.Length >= 18)
                    {
                        parseValue = BitConverter.ToSingle(raw, 4);
                        parseName = System.Text.Encoding.ASCII.GetString(raw, 8, 10).TrimEnd('\0');
                        diagType += "→raw";
                    }
                    // 方式2: 类型化结构体 (MP 支持的话)
                    else
                    {
                        try
                        {
                            var nvf = (MAVLink.mavlink_named_value_float_t)msg.data;
                            parseValue = nvf.value;
                            byte[] nameBytes = new byte[10];
                            for (int i = 0; i < 10; i++) nameBytes[i] = nvf.name[i];
                            parseName = System.Text.Encoding.ASCII.GetString(nameBytes).TrimEnd('\0');
                            diagType += "→typed";
                        }
                        catch
                        {
                            // 反射兜底
                            if (dt != null)
                            {
                                var vf = dt.GetField("value");
                                if (vf != null) parseValue = Convert.ToSingle(vf.GetValue(msg.data));
                                var nf = dt.GetField("name");
                                if (nf != null)
                                {
                                    var nv = nf.GetValue(msg.data);
                                    if (nv is byte[] nb) parseName = System.Text.Encoding.ASCII.GetString(nb).TrimEnd('\0');
                                    else if (nv is char[] cn) parseName = new string(cn).TrimEnd('\0');
                                    else if (nv is string s) parseName = s.TrimEnd('\0');
                                }
                            }
                            diagType += "→reflection";
                        }
                    }
                }
                catch (Exception ex)
                {
                    parseName = "__ERR__";
                    diagType += " err:" + ex.Message;
                }

                string finalDiag = string.Format("PKT251 type={0} name='{1}' val={2:F2}", diagType, parseName ?? "null", parseValue);

                if (parseName != null && parseName.StartsWith("PIDR"))
                {
                    string cleanName = parseName.TrimEnd('\0').Trim();
                    if (cleanName.Length >= 5 && int.TryParse(cleanName.Substring(4), out int idx) && idx >= 0 && idx < 10)
                    {
                        lock (_pidrLock)
                        {
                            // 位掩码方案: 收到 PIDR0 时重置, 防止旧计数器被重复消息打乱
                            if (idx == 0) _pidrRecvMask = 0;
                            _pidrAccum[idx] = parseValue;
                            _pidrRecvMask |= (1 << idx);
                            int curMask = _pidrRecvMask;

                            if (curMask == 0x3FF)  // 全部 10 个都已收到
                            {
                                float[] data = new float[10];
                                Array.Copy(_pidrAccum, data, 10);
                                _pidrStaFlags = 0;
                                _pidrRecvMask = 0;  // 为下次查询准备
                                HandlePidResponse(data);
                                finalDiag = "PIDR 完成, 已更新控件 ✓";
                                // 按钮变绿反馈
                                this.BeginInvoke((Action)(() =>
                                {
                                    _btnQueryPid.BackColor = Color.FromArgb(0, 200, 80);
                                    _btnQueryPid.Text = "✓ 已同步";
                                }));
                            }
                            else
                            {
                                finalDiag = string.Format("PIDR {0}/10 [{1}={2:F2}]", CountBits(curMask), cleanName, parseValue);
                                this.BeginInvoke((Action)(() =>
                                {
                                    _btnQueryPid.BackColor = Color.FromArgb(255, 180, 0);
                                    _btnQueryPid.Text = string.Format("...{0}/10", CountBits(curMask));
                                }));
                            }
                        }
                    }
                }

                string fd = finalDiag;
                this.BeginInvoke((Action)(() => { _lblPidStatus.Text = fd; }));
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
                    _rxStatusLabel.Text = string.Format("数据正常 [{0}] | 总包:{1} 伺服:{2} RC:{3} SYS:{6} ST:{4} NVF:{5}",
                        _msgSource, _totalPacketCount, _servoPacketCount, _rcPacketCount,
                        _statustextPacketCount, _nvfPacketCount, _sysStatusPacketCount);
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
                    _rxStatusLabel.Text = string.Format("收到伺服:{0} RC:{1} 但数值为0 | 检查通道 ST:{2} NVF:{3}",
                        _servoPacketCount, _rcPacketCount, _statustextPacketCount, _nvfPacketCount);
                else if (_totalPacketCount == 0)
                    _rxStatusLabel.Text = "等待飞控数据... (确认飞控已连接)";
                else
                    _rxStatusLabel.Text = "等待飞控数据...";
            }
            // PID诊断: 如果正在等待回传, 显示底层消息统计
            if (_lblPidStatus != null && _lblPidStatus.Text.StartsWith("已发送查询"))
            {
                _lblPidStatus.Text = string.Format("已发送查询, 等待回传... | 总包:{0} ST:{1} NVF:{2}",
                    _totalPacketCount, _statustextPacketCount, _nvfPacketCount);
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
