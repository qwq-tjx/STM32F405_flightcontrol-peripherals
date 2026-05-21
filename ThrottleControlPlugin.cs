using System;
using System.Drawing;
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

        public override string Name => "四电机油门控制器";
        public override string Version => "1.0";
        public override string Author => "Your Name";

        public override bool Init() => true;

        public override bool Loaded()
        {
            _controlForm = new ThrottleControlForm();
            _controlForm.Show(Host.MainForm);
            return true;
        }

        public override bool Exit()
        {
            _controlForm?.Close();
            _controlForm?.Dispose();
            return true;
        }

        public override bool Loop() => true;
    }

    public class ThrottleControlForm : Form
    {
        private TrackBar[] trackBars = new TrackBar[4];
        private NumericUpDown[] numericUpDowns = new NumericUpDown[4];
        private Label[] labels = new Label[4];
        private Button btnSyncAll, btnEmergencyStop;
        private System.Windows.Forms.Timer _debounceTimer;
        private int[] _pendingValues = new int[4];
        private bool[] _pendingChannels = new bool[4];
        private DateTime[] _lastSendTime = new DateTime[4];
        private bool _suppressEvents;

        public ThrottleControlForm()
        {
            InitializeComponents();
            _debounceTimer = new System.Windows.Forms.Timer { Interval = 50 };
            _debounceTimer.Tick += OnDebounceTick;
        }

        private void InitializeComponents()
        {
            this.Text = "四电机油门控制器";
            this.Size = new Size(870, 280);
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
                    Text = $"电机 {i + 1}",
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

                this.Controls.Add(labels[i]);
                this.Controls.Add(trackBars[i]);
                this.Controls.Add(numericUpDowns[i]);
            }

            btnSyncAll = new Button
            {
                Text = "同步所有电机",
                Location = new Point(20, 140),
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
                Location = new Point(220, 140),
                Size = new Size(180, 35),
                BackColor = Color.FromArgb(220, 53, 69),
                ForeColor = Color.White,
                FlatStyle = FlatStyle.Flat,
                Font = new Font("微软雅黑", 9, FontStyle.Bold)
            };
            btnEmergencyStop.Click += (s, e) => SetAllThrottle(0);

            this.Controls.Add(btnSyncAll);
            this.Controls.Add(btnEmergencyStop);
        }

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
                        Console.WriteLine($"Error ch{i}: {ex.Message}");
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

            // 频率限制：同一通道 50ms 内只发一次
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
                    Console.WriteLine($"Error: {ex.Message}");
                }
            });
        }
    }
}
