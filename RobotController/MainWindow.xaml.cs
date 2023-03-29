﻿using System;
using System.Windows;
using Key = System.Windows.Input.Key;
using OpenRCF;
using System.Threading;
using System.Runtime.InteropServices;
using static OpenRCF.SerialDevice;
using static OpenRCF.Mobile;
using System.IO.Ports;
using SharpDX.XInput;

using GamepadButtonFlags = SharpDX.XInput.GamepadButtonFlags;

namespace RobotController   
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        public bool TorqueON = false;

        public bool DyWrPoSW = false; //DynamixelWritePositionSwitch

        public static uint CRANEX7_8_OpenRCF = 7; //8
        public uint BW = 3;
        public uint BW2 = 2;

        Simulator Simulator = new Simulator();

        Robot CRANE_X7 = new Robot(new uint[2] { 6, CRANEX7_8_OpenRCF }); // old 3

        SerialDevice.Dynamixel CRANE_X7_DXL = new SerialDevice.Dynamixel(1000000);
        byte[] ID0209 = new byte[8] { 2, 3, 4, 5, 6, 7, 8, 9 };
        //byte[] ID0209 = new byte[4] { 11, 12, 13, 14};

       
        public MainWindow()
        {
            InitializeComponent();
            Loaded += InitializeOpenRCF;

            Joystick.ButtonEvent[GamepadButtonFlags.A] = GetJoystick;
            Joystick.ButtonEvent[GamepadButtonFlags.DPadLeft] = GetJoystick2;

            string[] ports = SerialPort.GetPortNames();

            foreach (string port in ports)
            {
                PortList.Items.Add(port);
            }

            CRANE_X7.SetFloatingJoint6DOF(0.25f, 0.25f, 0.1f);

            CRANE_X7[0, 0].q = 0;// x axis possition
            CRANE_X7[0, 1].q = 0;
            CRANE_X7[0, 2].q = 0;
            //CRANEX7[0, 3].q = (float)((float)-90 * Math.PI / 180);// x axis rotate
            //CRANEX7[0, 4].q = (float)((float)90 * Math.PI / 180); // 1.5708f;
            //CRANEX7[0, 5].q = (float)((float)0 * Math.PI / 180);
            CRANE_X7[0, 3].q = (float)((float)90 * Math.PI / 180);// x axis rotate
            CRANE_X7[0, 4].q = (float)((float)-90 * Math.PI / 180); // 1.5708f;
            CRANE_X7[0, 5].q = (float)((float)180 * Math.PI / 180);

            // CRANEX7.SetPlanarJoint3DOF(0.25f, 0.25f, 0.1f);

            CRANE_X7.Kinematics.BasePosition[0] = -0.5f;
            CRANE_X7.Kinematics.BasePosition[1] = -0.25f;
            CRANE_X7.Kinematics.BasePosition[2] = 1.00f;

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

            CRANE_X7[1, 0].JointRange[0] = CRANE_X7_DXL.DyAngle2rad4servo(300);
            CRANE_X7[1, 0].JointRange[1] = CRANE_X7_DXL.DyAngle2rad4servo(3800);

            CRANE_X7[1, 1].JointRange[0] = CRANE_X7_DXL.DyAngle2rad4servo(1024);
            CRANE_X7[1, 1].JointRange[1] = CRANE_X7_DXL.DyAngle2rad4servo(3072);

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

            CRANE_X7[0, 0].qHome = 0;// x axis possition
            CRANE_X7[0, 1].qHome = 0;
            CRANE_X7[0, 2].qHome = 0;
            CRANE_X7[0, 3].qHome = (float)((float)90 * Math.PI / 180);// x axis rotate
            CRANE_X7[0, 4].qHome = (float)((float)-90 * Math.PI / 180); // 1.5708f;
            CRANE_X7[0, 5].qHome = (float)((float)180 * Math.PI / 180);
            CRANE_X7[1, 0].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2468);
            CRANE_X7[1, 1].qHome = CRANE_X7_DXL.DyAngle2rad4servo(3070);
            CRANE_X7[1, 2].qHome = CRANE_X7_DXL.DyAngle2rad4servo(3050);
            CRANE_X7[1, 3].qHome = CRANE_X7_DXL.DyAngle2rad4servo(954);
            CRANE_X7[1, 4].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2055);
            CRANE_X7[1, 5].qHome = CRANE_X7_DXL.DyAngle2rad4servo(1989);
            CRANE_X7[1, 6].qHome = CRANE_X7_DXL.DyAngle2rad4servo(2094);


            CRANE_X7.Kinematics.Target[1].Priority = false;

            CRANE_X7.Kinematics.ForwardKinematics();
            CRANE_X7.Kinematics.SetJointLinkRadiusAuto();     //defo0.2
            CRANE_X7.Kinematics.ResetTargets();

            for (uint linki = 0; linki < CRANEX7_8_OpenRCF; linki++)
            {
                CRANE_X7[1, linki].Link.Color.SetWhite();
                CRANE_X7[1, linki].Joint.Color.SetBlue();
            }

            Parallel.RunEndless(DyWritePos, 50);
            Parallel.RunEndless(Joystick.Main, 50);
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
        }

        private void OpenPort_Click(object sender, RoutedEventArgs e)
        {
            Console.WriteLine(OpenPort.Content.ToString());

            if (PortList.SelectedItem != null)
            {
                if(OpenPort.Content.ToString() == "Open") {

                    OpenPort.IsEnabled = false;

                    CRANE_X7_DXL.PortOpen(PortList.SelectedItem.ToString());
                    CRANE_X7_DXL.PositionControlMode(ID0209);

                    CRANE_X7_DXL.TorqueEnable(ID0209);

                    TorqueON = true;

                    OpenPort.Content = "Close";

                    OpenPort.IsEnabled = true;
                }
                else if(OpenPort.Content.ToString() == "Close")
                {
                    OpenPort.IsEnabled = false;

                    CRANE_X7_DXL.TorqueDisable(ID0209);
                    CRANE_X7_DXL.PortClose();

                    TorqueON = false;

                    OpenPort.Content = "Open";

                    OpenPort.IsEnabled = true;
                }
            }
            else { Console.WriteLine("Please select the PORT first"); }
        }

        public void DyWritePos()
        {
            if (DyWrPoSW == true)
            {
                for (byte i = 0; i < (byte)(CRANEX7_8_OpenRCF); i++)
                {
                    CRANE_X7_DXL.WritePosition((byte)(ID0209[0] + i), (int)CRANE_X7_DXL.rad2DyAngle4servo(CRANE_X7[1, i].q));
                    Console.WriteLine((int)CRANE_X7_DXL.rad2DyAngle4servo(CRANE_X7[1, i].q));
                }
            }
        }

        public void GetJoystick()
        {
            Console.WriteLine("A");
        }

        public void GetJoystick2()
        {
            Console.WriteLine("Left");
        }

        private void Button1_Click(object sender, RoutedEventArgs e)
        {
            //thread1.Start();
            CRANE_X7.Kinematics.ResetHomePosition();
            CRANE_X7.Kinematics.ForwardKinematics();
        }

        private void Button2_Click(object sender, RoutedEventArgs e)
        {
            CRANE_X7.Kinematics.Target[1].Position[0] = -0.8f;
            CRANE_X7.Kinematics.Target[1].Position[1] = 0;
            CRANE_X7.Kinematics.Target[1].Position[2] = 0.6f;
            //CRANEX7.Kinematics.Target[1].Rotate.SetRy(0.5*Math.PI);

            CRANE_X7.Kinematics.InverseKinematics();

            CRANE_X7.Trajectory.SetNode(1, 0);
            CRANE_X7.Trajectory.GetNode(0);
        }

        private void Button3_Click(object sender, RoutedEventArgs e)
        {
            DyWrPoSW = true;

            CRANE_X7.Trajectory.ProceedOutboundLine();
            CRANE_X7.Trajectory.SpeedRatio = 1;

            Parallel.Run(CRANE_X7.Trajectory.ProceedOutboundLine, 50);
        }

        private void Button4_Click(object sender, RoutedEventArgs e)
        {
            CRANE_X7.Trajectory.ProceedInboundLine();
            CRANE_X7.Trajectory.SpeedRatio = 1;

            Parallel.Run(CRANE_X7.Trajectory.ProceedInboundLine, 50);
        }

        private void Button5_Click(object sender, RoutedEventArgs e)
        {

        }

    }
}
