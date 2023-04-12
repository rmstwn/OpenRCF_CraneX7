using System;
using System.Collections.Generic;
using SharpDX.XInput;
using static System.Windows.Forms.AxHost;
using static OpenRCF.RobotObject;
using static OpenRCF.Mobile;
using static OpenRCF.SerialDevice;

using System.Threading;
using System.Windows.Controls.Primitives;

namespace OpenRCF
{
    public static class Joystick
    {
        public static Dictionary<GamepadButtonFlags, Action> ButtonEvent = new Dictionary<GamepadButtonFlags, Action>()
        {
            { GamepadButtonFlags.A, () => { } },
            { GamepadButtonFlags.B, () => { } },
            { GamepadButtonFlags.X, () => { } },
            { GamepadButtonFlags.Y, () => { } },
            { GamepadButtonFlags.DPadUp, () => { } },
            { GamepadButtonFlags.DPadDown, () => { } },
            { GamepadButtonFlags.DPadLeft, () => { } },
            { GamepadButtonFlags.DPadRight, () => { } },
            { GamepadButtonFlags.Start, () => { } },
            { GamepadButtonFlags.Back, () => { } },
            { GamepadButtonFlags.RightShoulder, () => { } },
            { GamepadButtonFlags.LeftShoulder, () => { } }
        };

        public static Dictionary<Gamepad, short[]> AnalogEvent = new Dictionary<Gamepad, short[]>()
        {

        };

        public class Data
        {
            public static short LeftThumbX;
            public static short LeftThumbY;
            public static short RightThumbX;
            public static short RightThumbY;
        }

        public static void GetJoystick()
        {
            //double[] TargetOdom = { 0, 0, 0 };
            //double[] vel = { 0, 0, 0, 0 };
            //int[] TargetVel = { 0, 0, 0, 0 };
            //int[] CurrentVel = { 0, 0, 0, 0 };

            //SerialDevice.Dynamixel Dynamixel = new SerialDevice.Dynamixel(1000000);

            //byte[] id = new byte[4] { 13, 12, 14, 11 }; //Mecanum
            ////byte[] id = new byte[4] { 11, 12, 13, 14 }; //Omni

            //Dynamixel.PortOpen("COM6");
            //Dynamixel.TorqueEnable(id);

            //MobileInfo MobileInfo = new MobileInfo();

            //JointState JointState = new JointState();

            Console.WriteLine("Start XGamepadApp");
            
            // Initialize XInput
            var controllers = new[] { new Controller(UserIndex.One), new Controller(UserIndex.Two), new Controller(UserIndex.Three), new Controller(UserIndex.Four) };

            // Get 1st controller available
            Controller controller = null;
            foreach (var selectControler in controllers)
            {
                if (selectControler.IsConnected)
                {
                    controller = selectControler;
                    break;
                }
            }

            if (controller == null)
            {
                Console.WriteLine("No XInput controller installed");
            }
            else
            {
                Console.WriteLine("Found a XInput controller available");
                Console.WriteLine("Press buttons on the controller to display events or escape key to exit... ");

                // Poll events from joystick
                var previousState = controller.GetState();
                while (controller.IsConnected)
                {

                    if (IsKeyPressed(ConsoleKey.Escape))
                    {
                        break;
                    }
                    var state = controller.GetState();

                    switch (state.Gamepad.Buttons) 
                    {
                        case GamepadButtonFlags.A:
                            ButtonEvent[GamepadButtonFlags.A].Invoke();
                            break;
                        case GamepadButtonFlags.B:
                            ButtonEvent[GamepadButtonFlags.B].Invoke();
                            break;
                        case GamepadButtonFlags.X:
                            ButtonEvent[GamepadButtonFlags.X].Invoke();
                            break;
                        case GamepadButtonFlags.Y:
                            ButtonEvent[GamepadButtonFlags.X].Invoke();
                            break;
                        case GamepadButtonFlags.DPadUp:
                            ButtonEvent[GamepadButtonFlags.DPadUp].Invoke();
                            break;
                        case GamepadButtonFlags.DPadDown:
                            ButtonEvent[GamepadButtonFlags.DPadDown].Invoke();
                            break;
                        case GamepadButtonFlags.DPadLeft:
                            ButtonEvent[GamepadButtonFlags.DPadLeft].Invoke();
                            break;
                        case GamepadButtonFlags.DPadRight:
                            ButtonEvent[GamepadButtonFlags.DPadRight].Invoke();
                            break;
                        case GamepadButtonFlags.Start:
                            ButtonEvent[GamepadButtonFlags.Start].Invoke();
                            break;
                        case GamepadButtonFlags.Back:
                            ButtonEvent[GamepadButtonFlags.Back].Invoke();
                            break;
                        case GamepadButtonFlags.RightShoulder:
                            ButtonEvent[GamepadButtonFlags.RightShoulder].Invoke();
                            break;
                        case GamepadButtonFlags.LeftShoulder:
                            ButtonEvent[GamepadButtonFlags.LeftShoulder].Invoke();
                            break;
                    }

                    Data.LeftThumbX = state.Gamepad.LeftThumbX;
                    Data.LeftThumbY = state.Gamepad.LeftThumbY;
                    Data.RightThumbX = state.Gamepad.RightThumbX;
                    Data.RightThumbY = state.Gamepad.RightThumbY;

                    //Console.WriteLine(state.Gamepad.LeftThumbX);

                    //if (previousState.PacketNumber != state.PacketNumber)
                    //    Console.WriteLine(state.Gamepad

                    //Console.WriteLine(state.Gamepad.Buttons);

                    //switch (state.Gamepad.Buttons)
                    //{
                    //    case GamepadButtonFlags.A:
                    //        //Mobile.Mecanum.MovePos(Dynamixel, id, 1000000, 0.1, new double[] { 1, 1, 0 });
                    //        //Console.WriteLine("Test");
                    //        GamepadButtonFlags.A;
                    //        break;
                    //        //    case GamepadButtonFlags.DPadUp:
                    //        //        Mobile.Mecanum.Move(Dynamixel, id, 1000000, new double[] { 0.43, 0.43, 0 });
                    //        //        break;
                    //        //    case GamepadButtonFlags.DPadDown:
                    //        //        Mobile.Mecanum.Move(Dynamixel, id, 1000000, new double[] { -0.43, 0, 0 });
                    //        //        break;
                    //        //    case GamepadButtonFlags.DPadLeft:
                    //        //        Mobile.Mecanum.Move(Dynamixel, id, 1000000, new double[] { 0, 0.43, 0 });
                    //        //        break;
                    //        //    case GamepadButtonFlags.DPadRight:
                    //        //        Mobile.Mecanum.Move(Dynamixel, id, 1000000, new double[] { 0, -0.43, 0 });
                    //        //        break;
                    //        //    case GamepadButtonFlags.LeftShoulder:
                    //        //        Mobile.Mecanum.Move(Dynamixel, id, 1000000, new double[] { 0.214, 0.214, 0 });
                    //        //        break;
                    //        //    case GamepadButtonFlags.RightShoulder:
                    //        //        Mobile.Mecanum.Move(Dynamixel, id, 1000000, new double[] { 0.214, -0.214, 0 });
                    //        //        break;
                    //        //    default:

                    //        //        state.Gamepad.Buttons

                    //        //        TargetOdom[0] = map(state.Gamepad.LeftThumbY, -32768, 32767, -0.43, 0.43);
                    //        //        TargetOdom[1] = map(state.Gamepad.LeftThumbX, -32768, 32767, 0.43, -0.43);
                    //        //        TargetOdom[2] = map(state.Gamepad.RightThumbX, -32768, 32767, -0.6, 0.6);

                    //        //        if (TargetOdom[0] < 0.1 && TargetOdom[0] > -0.1) TargetOdom[0] = 0;
                    //        //        if (TargetOdom[1] < 0.1 && TargetOdom[1] > -0.1) TargetOdom[1] = 0;
                    //        //        if (TargetOdom[2] < 0.1 && TargetOdom[2] > -0.1) TargetOdom[2] = 0;

                    //        //        Mobile.Mecanum.Move(Dynamixel, id, 1000000, TargetOdom);

                    //        //        break;
                    //        //}

                    Thread.Sleep(10);
                    previousState = state;
                }
            }
            Console.WriteLine("End XGamepadApp");
        }
        public static void GetJoystickRun()
        {
            Console.WriteLine("Test");
        }

        public static bool IsKeyPressed(ConsoleKey key)
        {
            return Console.KeyAvailable && Console.ReadKey(true).Key == key;
        }

        public static double map(double x, double in_min, double in_max, double out_min, double out_max)
        {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
    }
}
