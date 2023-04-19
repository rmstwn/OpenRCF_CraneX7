using OpenRCF;
using System;
using System.IO.Ports;
using System.Security.AccessControl;
using System.Windows;
using GamepadButtonFlags = SharpDX.XInput.GamepadButtonFlags;
using Vector = OpenRCF.Vector;

namespace RobotController
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {

        Vector IdouXYZ = new Vector(3);
        public static float IdouX, IdouY, IdouZ = 0;

        private const int V = 0;

        public bool TorqueON = false;
        public bool DyWrPoSW = false; //DynamixelWritePositionSwitch
        public bool CraneX7Connected = false; //CraneX7 Connected
        public bool CraneX7PortOpen = false; //Port Open

        public bool MoveMobileManual = false; //Set 
        public bool RotateRightState = false; //Set Rotate Right
        public bool RotateLeftState = false; //Set Rotate Left

        public static uint CRANEX7_8_OpenRCF = 7; //8
        public uint BW = 3;
        public uint BW2 = 2;

        float[] TargetPosByJoystick = new float[3] { 0, 0, 0 };
        float[] ClampedTarget = new float[3] { 0, 0, 0 };
        float[] HomeEoEPos = new float[3] { 0, 0, 0 };
        float[] CurrentEoEPos = new float[3] { 0, 0, 0 };

        Simulator Simulator = new Simulator();

        Robot CRANE_X7 = new Robot(new uint[2] { 3, CRANEX7_8_OpenRCF }); // old 3

        SerialDevice.Dynamixel CRANE_X7_DXL = new SerialDevice.Dynamixel(1000000);
        string CRANE_X7_Port = "COM7"; //MiniPC
                                       //string CRANE_X7_Port = "COM4"; //Laptop
        byte[] ID0209 = new byte[8] { 2, 3, 4, 5, 6, 7, 8, 9 };
        int[] angles = new int[8];

        //Mobile Definition
        SerialDevice.Dynamixel OmniRobot = new SerialDevice.Dynamixel(1000000);

        //byte[] id = new byte[4] { 13, 12, 14, 11 }; //Mecanum
        byte[] id = new byte[4] { 11, 12, 13, 14 }; //Omni

        double[] TargetOdom = { 0, 0, 0 };
        double[] vel = { 0, 0, 0, 0 };
        int[] TargetVel = { 0, 0, 0, 0 };
        int[] CurrentVel = { 0, 0, 0, 0 };

        double EuclideanDistance;
        double EuclideanDistanceBefore;

        float[] dTTemp = new float[3];

        //Cuboid Cuboid = new Cuboid(0.25f * 2, 0.25f * 2, 0.32f * 2);

        float[] elbowPosition = new float[3];

        public MainWindow()
        {
            InitializeComponent();
            Loaded += InitializeOpenRCF;

            //Cuboid.Position[1] = 0.31f;
            //Cuboid.Position[2] = 0.31f;

            Joystick.ButtonEvent[GamepadButtonFlags.A] = TorqueOn;
            //Joystick.ButtonEvent[GamepadButtonFlags.Y] = TorqueOff;
            Joystick.ButtonEvent[GamepadButtonFlags.Y] = CloseGripperFull;

            Joystick.ButtonEvent[GamepadButtonFlags.RightShoulder] = OpenGripper;
            Joystick.ButtonEvent[GamepadButtonFlags.LeftShoulder] = CloseGripperBall;

            Joystick.ButtonEvent[GamepadButtonFlags.DPadRight] = RotateRight;
            Joystick.ButtonEvent[GamepadButtonFlags.DPadLeft] = RotateLeft;

            Joystick.ButtonEvent[GamepadButtonFlags.DPadUp] = ArmUp;
            Joystick.ButtonEvent[GamepadButtonFlags.DPadDown] = ArmDown;

            Joystick.ButtonEvent[GamepadButtonFlags.Start] = BackToHomePos;

            // Mobile Def. start
            OmniRobot.PortOpen("COM6"); // MiniPC
                                        //OmniRobot.PortOpen("COM5"); // laptop 
            OmniRobot.VelocityControlMode(id);
            OmniRobot.TorqueEnable(id);

            MobileInfo MobileInfo = new MobileInfo();

            // Mobile Def. End

            string[] ports = SerialPort.GetPortNames();

            foreach (string port in ports)
            {
                PortList.Items.Add(port);
            }

            //CRANE_X7.SetFloatingJoint6DOF(0.25f, 0.25f, 0.1f);
            CRANE_X7.SetPlanarJoint3DOF(0.25f, 0.25f, 0.1f);

            //CRANE_X7[0, 0].q = 0;// x axis possition
            //CRANE_X7[0, 1].q = 0;
            //CRANE_X7[0, 2].q = 0;
            //CRANEX7[0, 3].q = (float)((float)-90 * Math.PI / 180);// x axis rotate
            //CRANEX7[0, 4].q = (float)((float)90 * Math.PI / 180); // 1.5708f;
            //CRANEX7[0, 5].q = (float)((float)0 * Math.PI / 180);
            //CRANE_X7[0, 3].q = (float)((float)90 * Math.PI / 180);// x axis rotate
            //CRANE_X7[0, 4].q = (float)((float)-90 * Math.PI / 180); // 1.5708f;
            //CRANE_X7[0, 5].q = (float)((float)180 * Math.PI / 180);

            // CRANEX7.SetPlanarJoint3DOF(0.25f, 0.25f, 0.1f);

            CRANE_X7.Kinematics.BasePosition[0] = 0f;
            CRANE_X7.Kinematics.BasePosition[1] = 0f;
            CRANE_X7.Kinematics.BasePosition[2] = 0f;

            CRANE_X7[1, 0].lInit.Set = new float[3] { 0, 0, 0 };
            CRANE_X7[1, 0].axisInit.SetUnitVectorZ();

            CRANE_X7[1, 1].lInit.Set = new float[3] { 0, 0, 0.05f * BW2 };
            CRANE_X7[1, 1].axisInit.SetUnitVectorX(-1);

            CRANE_X7[1, 2].lInit.Set = new float[3] { 0, 0, 0.05f * BW2 };
            CRANE_X7[1, 2].axisInit.SetUnitVectorZ();

            CRANE_X7[1, 3].lInit.Set = new float[3] { 0, 0, 0.20f * BW2 };
            CRANE_X7[1, 3].axisInit.SetUnitVectorX(-1);

            CRANE_X7[1, 4].lInit.Set = new float[3] { 0, 0, 0.135f * BW2 };
            CRANE_X7[1, 4].axisInit.SetUnitVectorZ();

            CRANE_X7[1, 5].lInit.Set = new float[3] { 0, 0, 0.115f * BW2 };
            CRANE_X7[1, 5].axisInit.SetUnitVectorX(-1);

            CRANE_X7[1, 6].lInit.Set = new float[3] { 0, 0, 0.02f * BW2 };
            CRANE_X7[1, 6].axisInit.SetUnitVectorZ();

            CRANE_X7[1, 7].lInit.Set = new float[3] { 0.02f, 0, 0 };
            CRANE_X7[1, 7].axisInit.SetUnitVectorX(-1);

            CRANE_X7[1, 0].JointRange[0] = CRANE_X7_DXL.DyAngle2rad4servo(1024);
            CRANE_X7[1, 0].JointRange[1] = CRANE_X7_DXL.DyAngle2rad4servo(3073);

            CRANE_X7[1, 1].JointRange[0] = CRANE_X7_DXL.DyAngle2rad4servo(1024);
            CRANE_X7[1, 1].JointRange[1] = CRANE_X7_DXL.DyAngle2rad4servo(2048);

            CRANE_X7[1, 2].JointRange[0] = CRANE_X7_DXL.DyAngle2rad4servo(300);
            CRANE_X7[1, 2].JointRange[1] = CRANE_X7_DXL.DyAngle2rad4servo(3800);

            CRANE_X7[1, 3].JointRange[0] = CRANE_X7_DXL.DyAngle2rad4servo(270);
            CRANE_X7[1, 3].JointRange[1] = CRANE_X7_DXL.DyAngle2rad4servo(2048);

            CRANE_X7[1, 4].JointRange[0] = CRANE_X7_DXL.DyAngle2rad4servo(300);
            CRANE_X7[1, 4].JointRange[1] = CRANE_X7_DXL.DyAngle2rad4servo(3800);

            CRANE_X7[1, 5].JointRange[0] = CRANE_X7_DXL.DyAngle2rad4servo(1024);
            CRANE_X7[1, 5].JointRange[1] = CRANE_X7_DXL.DyAngle2rad4servo(3072);

            CRANE_X7[1, 6].JointRange[0] = CRANE_X7_DXL.DyAngle2rad4servo(200);
            CRANE_X7[1, 6].JointRange[1] = CRANE_X7_DXL.DyAngle2rad4servo(3900);

            CRANE_X7[1, 7].JointRange[0] = CRANE_X7_DXL.DyAngle2rad4servo(2048);
            CRANE_X7[1, 7].JointRange[1] = CRANE_X7_DXL.DyAngle2rad4servo(3072);

            //CRANE_X7[0, 0].qHome = 0;// x axis possition
            //CRANE_X7[0, 1].qHome = 0;
            //CRANE_X7[0, 2].qHome = 0;
            //CRANE_X7[0, 3].qHome = (float)((float)90 * Math.PI / 180);// x axis rotate
            //CRANE_X7[0, 4].qHome = (float)((float)-90 * Math.PI / 180); // 1.5708f;
            //CRANE_X7[0, 5].qHome = (float)((float)180 * Math.PI / 180);
            //CRANE_X7[1, 0].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2312);
            //CRANE_X7[1, 1].qHome = CRANE_X7_DXL.DyAngle2rad4servo(3040);
            //CRANE_X7[1, 2].qHome = CRANE_X7_DXL.DyAngle2rad4servo(3103);
            //CRANE_X7[1, 3].qHome = CRANE_X7_DXL.DyAngle2rad4servo(1103);
            //CRANE_X7[1, 4].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2037);
            //CRANE_X7[1, 5].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2153); 
            //CRANE_X7[1, 6].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2031);

            //CRANE_X7[0, 0].qHome = 0;// x axis possition
            //CRANE_X7[0, 1].qHome = 0;
            //CRANE_X7[0, 2].qHome = 0;
            //CRANE_X7[0, 3].qHome = (float)((float)90 * Math.PI / 180);// x axis rotate
            //CRANE_X7[0, 4].qHome = (float)((float)-90 * Math.PI / 180); // 1.5708f;
            //CRANE_X7[0, 5].qHome = (float)((float)180 * Math.PI / 180);
            //CRANE_X7[1, 0].qHome = 0;
            //CRANE_X7[1, 1].qHome = 0;
            //CRANE_X7[1, 2].qHome = 0;
            //CRANE_X7[1, 3].qHome = 0;
            //CRANE_X7[1, 4].qHome = 0;
            //CRANE_X7[1, 5].qHome = 0;
            //CRANE_X7[1, 6].qHome = 0;

            //CRANEX7[1, 0].qHome = 0;
            //CRANEX7[1, 1].qHome = 0;
            //CRANEX7[1, 2].qHome = 0;
            //CRANEX7[1, 3].qHome = (float)(-0.2f * Math.PI);
            //CRANEX7[1, 4].qHome = 0;
            //CRANEX7[1, 5].qHome = 0;
            //CRANEX7[1, 6].qHome = 0;

            //CRANE_X7[0, 0].qHome = 0;// x axis possition
            //CRANE_X7[0, 1].qHome = 0;
            //CRANE_X7[0, 2].qHome = 0;
            //CRANE_X7[0, 3].qHome = (float)((float)90 * Math.PI / 180);// x axis rotate
            //CRANE_X7[0, 4].qHome = (float)((float)-90 * Math.PI / 180); // 1.5708f;
            //CRANE_X7[0, 5].qHome = (float)((float)180 * Math.PI / 180);
            //CRANE_X7[1, 0].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2468);
            //CRANE_X7[1, 1].qHome = CRANE_X7_DXL.DyAngle2rad4servo(3070);
            //CRANE_X7[1, 2].qHome = CRANE_X7_DXL.DyAngle2rad4servo(3050);
            //CRANE_X7[1, 3].qHome = CRANE_X7_DXL.DyAngle2rad4servo(954);
            //CRANE_X7[1, 4].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2055);
            //CRANE_X7[1, 5].qHome = CRANE_X7_DXL.DyAngle2rad4servo(1989);
            //CRANE_X7[1, 6].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2094);

            CRANE_X7[1, 0].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2066);
            CRANE_X7[1, 1].qHome = CRANE_X7_DXL.DyAngle2rad4servo(1773);
            CRANE_X7[1, 2].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2040);
            CRANE_X7[1, 3].qHome = CRANE_X7_DXL.DyAngle2rad4servo(723);
            CRANE_X7[1, 4].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2111);
            CRANE_X7[1, 5].qHome = CRANE_X7_DXL.DyAngle2rad4servo(1622);
            CRANE_X7[1, 6].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2108);
            CRANE_X7[1, 7].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2040);
            //2066,1773,2040,723,2111,1622,916,2040

            CRANE_X7.Kinematics.Target[1].Priority = false;

            CRANE_X7.Kinematics.ForwardKinematics();
            CRANE_X7.Kinematics.SetJointLinkRadiusAuto();     //defo0.2
            CRANE_X7.Kinematics.ResetTargets();

            for (uint linki = 0; linki < CRANEX7_8_OpenRCF; linki++)
            {
                CRANE_X7[1, linki].Link.Color.SetWhite();
                CRANE_X7[1, linki].Joint.Color.SetBlue();
            }

            HomeEoEPos[0] = CRANE_X7.Kinematics.Chain[1].pe[0];
            HomeEoEPos[1] = CRANE_X7.Kinematics.Chain[1].pe[1];
            HomeEoEPos[2] = CRANE_X7.Kinematics.Chain[1].pe[2];

            TargetPosByJoystick[0] = HomeEoEPos[0];
            TargetPosByJoystick[1] = HomeEoEPos[1];
            TargetPosByJoystick[2] = HomeEoEPos[2];

            CraneX7PortOpen = false;

            CRANE_X7_Initialize();

            Parallel.RunEndless(Joystick.GetJoystick, 50);
            Parallel.RunEndless(DyWritePos, 50);
            Parallel.RunEndless(SetTargetByJoystick, 50);
            Parallel.RunEndless(SetMobileTargetByJoystick, 20);

            //Parallel.RunEndless(CheckCol, 50);
            //Parallel.RunEndless(RotateRight, 50);
            //Parallel.RunEndless(RotateLeft, 50);


        }

        void InitializeOpenRCF(object sender, RoutedEventArgs e)
        {
            Core.SetFPS(30);
            Core.SetDrawFunction = Draw;
            Simulator.Owner = this;
            Simulator.Show();
        }

        private void Draw()
        {
            CRANE_X7.Draw();
            //Cuboid.Draw();
        }

        private void OpenPort_Click(object sender, RoutedEventArgs e)
        {
            Console.WriteLine(OpenPort.Content.ToString());

            if (PortList.SelectedItem != null)
            {
                if (OpenPort.Content.ToString() == "Open")
                {

                    OpenPort.IsEnabled = false;

                    CRANE_X7_DXL.PortOpen(PortList.SelectedItem.ToString());
                    CRANE_X7_DXL.PositionControlMode(ID0209);

                    CRANE_X7_DXL.TorqueEnable(ID0209);

                    CloseGripperFull();

                    TorqueON = true;

                    OpenPort.Content = "Close";
                    OpenPort.IsEnabled = true;

                    CraneX7Connected = true;
                }
                else if (OpenPort.Content.ToString() == "Close")
                {
                    OpenPort.IsEnabled = false;

                    CloseGripperFull();

                    CRANE_X7_DXL.TorqueDisable(ID0209);
                    CRANE_X7_DXL.PortClose();

                    TorqueON = false;

                    OpenPort.Content = "Open";
                    OpenPort.IsEnabled = true;

                    CraneX7Connected = false;
                }
            }
            else { Console.WriteLine("Please select the PORT first"); }
        }

        private void CRANE_X7_Initialize()
        {
            CraneX7Connected = false;
            DyWrPoSW = false;
            MoveMobileManual = false;

            if (CraneX7PortOpen == false)
            {
                CRANE_X7_DXL.PortOpen(CRANE_X7_Port);
            }
            CRANE_X7_DXL.PositionControlMode(ID0209);
            CRANE_X7_DXL.TorqueEnable(ID0209);

            CloseGripperFull();

            //if (CraneX7Connected == true)
            //{
            //    CRANE_X7_DXL.RequestPositionReply(ID0209);
            //    int[] CurrPos = CRANE_X7_DXL.Position(ID0209);
            //}

            CRANE_X7.Kinematics.Target[1].Position[0] = HomeEoEPos[0];
            CRANE_X7.Kinematics.Target[1].Position[1] = HomeEoEPos[1];
            CRANE_X7.Kinematics.Target[1].Position[2] = HomeEoEPos[2];

            CRANE_X7.Kinematics.InverseKinematics();

            //CRANE_X7.Trajectory.SetNode(1, 0);
            //CRANE_X7.Trajectory.GetNode(0);

            //for (byte i = 0; i < ID0209.Length - 1; i++) { 

            //}
            for (byte i = 0; i < ID0209.Length - 1; i++)
            {
                angles[i] = (int)CRANE_X7_DXL.rad2DyAngle4servo(CRANE_X7[1, i].q);
            }

            //CRANE_X7.Trajectory.ProceedInboundLine();
            //CRANE_X7.Trajectory.SpeedRatio = 1;

            //Parallel.Run(CRANE_X7.Trajectory.ProceedInboundLine, 50);

            CRANE_X7_DXL.WritePosition(ID0209, angles);

            CraneX7PortOpen = true;
            TorqueON = true;
            CraneX7Connected = true;
            DyWrPoSW = true;
            MoveMobileManual = true;
        }

        public void BackToHomePos()
        {
            CloseGripperFull();

            CRANE_X7.Kinematics.Target[1].Position[0] = HomeEoEPos[0];
            CRANE_X7.Kinematics.Target[1].Position[1] = HomeEoEPos[1];
            CRANE_X7.Kinematics.Target[1].Position[2] = HomeEoEPos[2];

            CRANE_X7.Kinematics.InverseKinematics();
        }

        public void DyWritePos()
        {
            if (DyWrPoSW == true)
            {
                for (byte i = 0; i < ID0209.Length - 1; i++)
                {
                    angles[i] = (int)CRANE_X7_DXL.rad2DyAngle4servo(CRANE_X7[1, i].q);
                }

                CRANE_X7_DXL.WritePosition(ID0209, angles); // good b fast

                //for (int i = 0; i < id.Length; i++) CRANE_X7_DXL.WritePosition(id[i], angles[i]);  // bad slow

                //for (byte i = 0; i < (byte)(CRANEX7_8_OpenRCF); i++)
                //{
                //    CRANE_X7_DXL.WritePosition((byte)(ID0209[0] + i), (int)CRANE_X7_DXL.rad2DyAngle4servo(CRANE_X7[1, i].q));
                //    //Console.WriteLine((int)CRANE_X7_DXL.rad2DyAngle4servo(CRANE_X7[1, i].q));
                //}
            }

            if (CraneX7Connected == true)
            {

                CRANE_X7_DXL.RequestPositionReply(ID0209);
                //CRANE_X7_DXL.ConsoleWriteReceiveData(ID0209);

                int[] CurrPos = CRANE_X7_DXL.Position(ID0209);

                //for (byte i = 0; i < (byte)(CRANEX7_8_OpenRCF); i++)
                //{
                //    Console.Write("{0},", CurrPos[i]);
                //}
                //Console.WriteLine("{0}", CurrPos[CRANEX7_8_OpenRCF]);

            }
        }

        public void SetTargetByJoystick()
        {
            CRANE_X7.Kinematics.ForwardKinematics();

            //TargetPosByJoystick[0] = TargetPosByJoystick[0] + (float)map(Joystick.Data.LeftThumbX, -32768, 32767, -0.03, 0.03);
            //TargetPosByJoystick[1] = TargetPosByJoystick[1] + (float)map(Joystick.Data.LeftThumbY, -32768, 32767, -0.03, 0.03);
            //TargetPosByJoystick[2] = TargetPosByJoystick[2] + (float)map(Joystick.Data.RightThumbY, -32768, 32767, -0.03, 0.03);

            //EuclideanDistance = Math.Round(Math.Sqrt(Math.Pow(TargetPosByJoystick[0] - HomeEoEPos[0], 2) + Math.Pow(TargetPosByJoystick[1] - HomeEoEPos[1], 2) + Math.Pow(TargetPosByJoystick[2] - HomeEoEPos[2], 2)), 2);

            //Console.Write("EuDist = {0} , {1} || ", EuclideanDistance, EuclideanDistanceBefore);

            //if (EuclideanDistance > 0.4)
            //{
            //    CRANE_X7.Kinematics.Target[1].Position[0] = CRANE_X7.Kinematics.Target[1].Position[0];
            //    CRANE_X7.Kinematics.Target[1].Position[1] = CRANE_X7.Kinematics.Target[1].Position[1];
            //    CRANE_X7.Kinematics.Target[1].Position[2] = CRANE_X7.Kinematics.Target[1].Position[2];

            //}
            //else
            //{
            //    CRANE_X7.Kinematics.Target[1].Position[0] = TargetPosByJoystick[0];
            //    CRANE_X7.Kinematics.Target[1].Position[1] = TargetPosByJoystick[1];
            //    CRANE_X7.Kinematics.Target[1].Position[2] = TargetPosByJoystick[2];
            //}

            //CurrentEoEPos[0] = TargetPosByJoystick[0] - HomeEoEPos[0];
            //CurrentEoEPos[1] = TargetPosByJoystick[1] - HomeEoEPos[1];
            //CurrentEoEPos[2] = TargetPosByJoystick[2] - HomeEoEPos[2];

            TargetPosByJoystick[0] = TargetPosByJoystick[0] + (float)map(Joystick.Data.RightThumbX, -32768, 32767, 0.02, -0.02);
            TargetPosByJoystick[1] = TargetPosByJoystick[1] + (float)map(Joystick.Data.RightThumbY, -32768, 32767, 0.02, -0.02);
            //TargetPosByJoystick[2] = TargetPosByJoystick[2] + (float)map(Joystick.Data.RightThumbY, -32768, 32767, -0.01, 0.01);
            TargetPosByJoystick[2] = TargetPosByJoystick[2];

            //float normSq = (TargetPosByJoystick[0] * TargetPosByJoystick[0]) + (TargetPosByJoystick[1] * TargetPosByJoystick[1]) + (TargetPosByJoystick[2] * TargetPosByJoystick[2]);

            //if (normSq > (1 * 1))
            //{
            //    TargetOdom[0] = map(Joystick.Data.LeftThumbY, -32768, 32767, -0.3, 0.3);
            //    TargetOdom[1] = map(Joystick.Data.LeftThumbX, -32768, 32767, 0.3, -0.3);
            //    TargetOdom[2] = map(Joystick.Data.RightThumbX, -32768, 32767, -0.3, 0.3);

            //    if (TargetOdom[0] < 0.1 && TargetOdom[0] > -0.1) TargetOdom[0] = 0;
            //    if (TargetOdom[1] < 0.1 && TargetOdom[1] > -0.1) TargetOdom[1] = 0;
            //    if (TargetOdom[2] < 0.1 && TargetOdom[2] > -0.1) TargetOdom[2] = 0;

            //    Mobile.Omnidirectional.Move(OmniRobot, id, 1000000, TargetOdom);
            //}
            //else
            //{
            ClampedTarget = ClampTarget(TargetPosByJoystick, 1);

            CRANE_X7.Kinematics.Target[1].Position[0] = ClampedTarget[0];
            CRANE_X7.Kinematics.Target[1].Position[1] = ClampedTarget[1];
            CRANE_X7.Kinematics.Target[1].Position[2] = ClampedTarget[2];

            //Console.WriteLine("{0}, {1}, {2} || ", ClampedTarget[0], ClampedTarget[1], ClampedTarget[2]);
            //Console.WriteLine("{0}, {1}, {2}", TargetPosByJoystick[0], TargetPosByJoystick[1], TargetPosByJoystick[2]);

            CRANE_X7.Kinematics.InverseKinematics();
            //}

            int[] CommandPos = new int[8];

            for (byte i = 0; i < ID0209.Length; i++)
            {
                CommandPos[i] = (int)CRANE_X7_DXL.rad2DyAngle4servo(CRANE_X7[1, i].q);
            }

            //for (byte i = 0; i < (byte)(CRANEX7_8_OpenRCF - 1); i++)
            //{
            //    Console.Write("{0},", CommandPos[i]);
            //}
            //Console.WriteLine("{0}", CommandPos[CRANEX7_8_OpenRCF - 1]);

            //if (EuclideanDistanceBefore != EuclideanDistance) EuclideanDistanceBefore = EuclideanDistance;
        }

        public void SetMobileTargetByJoystick()
        {
            if (MoveMobileManual == true)
            {

                TargetOdom[0] = map(Joystick.Data.LeftThumbY, -32768, 32767, -0.5, 0.5);
                TargetOdom[1] = map(Joystick.Data.LeftThumbX, -32768, 32767, 0.5, -0.5);

                if(RotateRightState == true)
                {
                    TargetOdom[2] = 0.5;
                }
                else if (RotateLeftState == true)
                {
                    TargetOdom[2] = -0.5;
                }
                else
                {
                    TargetOdom[2] = 0;
                }

                if (TargetOdom[0] < 0.1 && TargetOdom[0] > -0.1) TargetOdom[0] = 0;
                if (TargetOdom[1] < 0.1 && TargetOdom[1] > -0.1) TargetOdom[1] = 0;
                if (TargetOdom[2] < 0.1 && TargetOdom[2] > -0.1) TargetOdom[2] = 0;

                Mobile.Omnidirectional.Move(OmniRobot, id, 1000000, TargetOdom);

            }

            RotateLeftState = false;
            RotateRightState = false;

            //Console.WriteLine(TargetOdom[0]);
        }

        public void RotateRight()
        {
            //MoveMobileManual = false;
            RotateRightState = true;

            //TargetOdom[0] = 0;
            //TargetOdom[1] = 0;
            //TargetOdom[2] = 0.5;

            //if (TargetOdom[0] < 0.1 && TargetOdom[0] > -0.1) TargetOdom[0] = 0;
            //if (TargetOdom[1] < 0.1 && TargetOdom[1] > -0.1) TargetOdom[1] = 0;
            //if (TargetOdom[2] < 0.1 && TargetOdom[2] > -0.1) TargetOdom[2] = 0;

            //Mobile.Omnidirectional.Move(OmniRobot, id, 1000000, TargetOdom);

            MoveMobileManual = true;
            //RotateRightState = false;
        }

        public void RotateLeft()
        {
            //MoveMobileManual = false;
            RotateLeftState = true;

            //TargetOdom[0] = 0;
            //TargetOdom[1] = 0;

            //TargetOdom[2] = - 0.5;

            //if (TargetOdom[0] < 0.1 && TargetOdom[0] > -0.1) TargetOdom[0] = 0;
            //if (TargetOdom[1] < 0.1 && TargetOdom[1] > -0.1) TargetOdom[1] = 0;
            //if (TargetOdom[2] < 0.1 && TargetOdom[2] > -0.1) TargetOdom[2] = 0;

            //Mobile.Omnidirectional.Move(OmniRobot, id, 1000000, TargetOdom);

            MoveMobileManual = true;
            //RotateLeftState = false;
        }

        public void ArmUp()
        {
            TargetPosByJoystick[2] = (float)(TargetPosByJoystick[2] + 0.003);
        }

        public void ArmDown()
        {
            TargetPosByJoystick[2] = (float)(TargetPosByJoystick[2] - 0.003);
        }

        public void OpenGripper()
        {
            CRANE_X7_DXL.WritePosition(ID0209[7], (int)CRANE_X7_DXL.rad2DyAngle4servo(1));
        }

        public void CloseGripperFull()
        {
            CRANE_X7_DXL.WritePosition(ID0209[7], 2048);
        }

        public void CloseGripperBall()
        {
            CRANE_X7_DXL.WritePosition(ID0209[7], 2350);
        }

        public void TorqueOn()
        {
            DyWrPoSW = true;
            CRANE_X7_DXL.TorqueEnable(ID0209);
            TorqueON = true;
        }

        public void TorqueOff()
        {
            DyWrPoSW = false;
            CRANE_X7_DXL.TorqueDisable(ID0209);
            TorqueON = false;
        }

        public float[] ClampTarget(float[] dS, double dSclamp)
        {
            float[] dT = new float[3];
            float normSq = (dS[0] * dS[0]) + (dS[1] * dS[1]) + (dS[2] * dS[2]);

            //elbowPosition[0] = CRANE_X7.Kinematics.Chain[1].Pair[3].pt[0];
            //elbowPosition[1] = CRANE_X7.Kinematics.Chain[1].Pair[3].pt[1];
            //elbowPosition[2] = CRANE_X7.Kinematics.Chain[1].Pair[3].pt[2];

            if (normSq > (dSclamp * dSclamp) || dS[1] >= (float)-0.25)
            {
                //float factor = (float)(dSclamp / Math.Sqrt(normSq));

                //if (normSq > (dSclamp * dSclamp))
                //{

                dS[0] = dTTemp[0];
                dS[1] = dTTemp[1];
                dS[2] = dTTemp[2];
                //}

                //dT[0] = dS[0] * factor;
                //dT[1] = dS[1] * factor;
                //dT[2] = dS[2] * factor;

                dT[0] = dS[0];
                dT[1] = dS[1];
                dT[2] = dS[2];

                //Console.WriteLine(factor);

                //TargetOdom[0] = map(Joystick.Data.RightThumbY, -32768, 32767, -0.5, 0.5);
                //TargetOdom[1] = map(Joystick.Data.RightThumbX, -32768, 32767, 0.5, -0.5);
                ////TargetOdom[2] = map(Joystick.Data.RightThumbX, -32768, 32767, -0.3, 0.3);
                //TargetOdom[2] = 0;

                //Mobile.Omnidirectional.Move(OmniRobot, id, 1000000, TargetOdom);

            }
            else
            {
                if (dS[1] > -0.25)
                {
                    dS[1] = (float)-0.25;
                }

                dT[0] = dS[0];
                dT[1] = dS[1];
                dT[2] = dS[2];

                MoveMobileManual = true;

                //TargetOdom[0] = 0;
                //TargetOdom[1] = 0;
                //TargetOdom[2] = 0;

                //Mobile.Omnidirectional.Move(OmniRobot, id, 1000000, TargetOdom);
            }

            if (normSq < (dSclamp * dSclamp))
            {
                if (dS[1] > -0.25)
                {
                    dS[1] = (float)-0.25;
                }

                if (elbowPosition[1] >= -0.01f)
                {
                    dS[0] = CRANE_X7.Kinematics.Chain[1].pe[0];
                    dS[1] = CRANE_X7.Kinematics.Chain[1].pe[1];
                    dS[2] = CRANE_X7.Kinematics.Chain[1].pe[2];
                }

                dTTemp[0] = dS[0];
                dTTemp[1] = dS[1];
                dTTemp[2] = dS[2];
            }

            //Console.WriteLine("{0}, {1}, {2} ", elbowPosition[0], elbowPosition[1], elbowPosition[2]);

            return dT;
        }

        public static double map(double x, double in_min, double in_max, double out_min, double out_max)
        {
            if (x == 128) return 0;
            else return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }

        private void Button1_Click(object sender, RoutedEventArgs e)
        {
            //thread1.Start();
            //CRANE_X7.Kinematics.ResetHomePosition();
            //CRANE_X7.Kinematics.ForwardKinematics();

            //CloseGripperFull();

            //CRANE_X7.Kinematics.Chain[1].pe.ConsoleWrite();
            // CRANE_X7.Kinematics.Chain[1].Re.ConsoleWrite();
        }

        private void Button2_Click(object sender, RoutedEventArgs e)
        {
            //CRANE_X7.Kinematics.Target[1].Position[0] = 0f;
            //CRANE_X7.Kinematics.Target[1].Position[1] = -0.6f;
            //CRANE_X7.Kinematics.Target[1].Position[2] = 0.6f;
            ////CRANEX7.Kinematics.Target[1].Rotate.SetRy(0.5*Math.PI);

            //CRANE_X7.Kinematics.InverseKinematics();

            //CRANE_X7.Trajectory.SetNode(1, 0);
            //CRANE_X7.Trajectory.GetNode(0);
        }

        private void Button3_Click(object sender, RoutedEventArgs e)
        {
            //DyWrPoSW = true;

            //CRANE_X7.Trajectory.ProceedOutboundLine();
            //CRANE_X7.Trajectory.SpeedRatio = 1;

            //Parallel.Run(CRANE_X7.Trajectory.ProceedOutboundLine, 50);
        }

        private void Button4_Click(object sender, RoutedEventArgs e)
        {
            //CRANE_X7.Trajectory.ProceedInboundLine();
            //CRANE_X7.Trajectory.SpeedRatio = 1;

            //Parallel.Run(CRANE_X7.Trajectory.ProceedInboundLine, 50);
        }

        private void Button5_Click(object sender, RoutedEventArgs e)
        {
            //    DyWrPoSW = true;
            //    CRANE_X7_DXL.TorqueEnable(ID0209);
            //    TorqueON = true;
        }

        private void Button6_Click(object sender, RoutedEventArgs e)
        {
            //DyWrPoSW = false;
            //CRANE_X7_DXL.TorqueDisable(ID0209);
            //TorqueON = false;
        }
    }
}
