

namespace ScanLineCamera
{
    using System;

    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.Windows.Threading;
    using System.Windows.Shapes;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Input;
    using Microsoft.Kinect;
    using AdvancedHMIDrivers;
    using MfgControl.AdvancedHMI;
    using System.Windows.Controls;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window
    {

    //  private const int MapDepthToByte = 8000 / 256;
        private const int MapDepthToByte = 256/256;
        private KinectSensor kinectSensor = null;
        private DepthFrameReader depthFrameReader = null;
        private ColorFrameReader _colorReader = null;
        private FrameDescription depthFrameDescription = null;
        private WriteableBitmap depthBitmap = null;
        private WriteableBitmap _bmp = null;
        private byte[] depthPixels = null;
        int[] Reddepthcall = null;
        int[] Bluedepthcall = null;
        int[] Greendepthcall = null;
        int[] Yellowdepthcall = null;
        ushort[] _depthData = null;
        int _depthHeight;
        int _depthWidth;
        int counter;
        int counter1;
        
        string Red_Dispaly_Result;
        string Blue_Dispaly_Result;
        string Green_Dispaly_Result;
        string Yellow_Dispaly_Result;
       
              

        EthernetIPforCLXComm CookieJar = new EthernetIPforCLXComm();
        DispatcherTimer timerReadPLC = new DispatcherTimer();
        DispatcherTimer timerReadMes = new DispatcherTimer();
        DispatcherTimer SliderLines = new DispatcherTimer();


         public MainWindow()
         {
             this.InitializeComponent();
             this.kinectSensor = KinectSensor.GetDefault();

             this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();         
             this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;
             this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
             this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
             this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);
           this.rgb.Source = depthBitmap;

             this._colorReader = this.kinectSensor.ColorFrameSource.OpenReader();
             this._colorReader.FrameArrived += this._colorReader_FrameArrived;      
             var description = _colorReader.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
             this._bmp = new WriteableBitmap(description.Width, description.Height, 96.0, 96.0, PixelFormats.Bgra32, null);
             this.rgblive.Source = _bmp;  
                     
             this.rgb.MouseLeftButtonUp += DepthImage_MouseLeftButtonUp;

             this.kinectSensor.Open();
             this._depthHeight = kinectSensor.DepthFrameSource.FrameDescription.Height;
             this._depthWidth = kinectSensor.DepthFrameSource.FrameDescription.Width;

             this.sliderYRed.Value = 150;
             this.sliderRed_L.Value = 0;
             this.sliderRed_R.Value = 512;
             this.sliderYBlue.Value = 225;
             this.sliderBlue_L.Value = 0;
             this.sliderBlue_R.Value = 512;     
             this.sliderXYellow.Value = 75;
             this.sliderYellow_T.Value = 0;
             this.sliderYellow_B.Value = 512;
             this.sliderXGrn.Value = 300;
             this.sliderGreen_T.Value = 0;
             this.sliderGreen_B.Value = 512;

             this.timerReadPLC.Interval = TimeSpan.FromMilliseconds(100); //every 200ms update
             this.timerReadPLC.Tick += new EventHandler(timerReadPLC_Tick);
             this.timerReadMes.Interval = TimeSpan.FromMilliseconds(100); //every 200ms update
             this.timerReadMes.Tick += new EventHandler(timerReadMes_Tick);
             this.SliderLines.Interval = TimeSpan.FromMilliseconds(100); //every 200ms update
             this.SliderLines.Tick += new EventHandler(Sliderlines_Tick);

             this.radioButton1.IsChecked = true;
             this. RGB_Label.Content = 1;

             this.RedPixelSet.Text = Settings1.Default.RedPixelSet;
             this.BluePixelSet.Text = Settings1.Default.BluePixelSet;
             this.GreenPixelSet.Text = Settings1.Default.GreenPixelSet;
            this.YellowPixelSet.Text = Settings1.Default.YellowPixelSet;

             this.IP_Address.Text = Settings1.Default.IP_Address;
             this.Low.Text = "1";
             this.HIGH1.Text = "2000";
             this.sliderGraph_High.Value = 2000;
             CookieJar.IPAddress = Settings1.Default.IP_Address; 
             rotation.Content = "90";
             amplification.Text = "10";
         }
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
                timerReadPLC.Stop();
            }
        }
        #region ImageProcessing
        private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    _depthData = new ushort[depthFrame.FrameDescription.LengthInPixels];
                    depthFrame.CopyFrameDataToArray(_depthData);

                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {

                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = ushort.MaxValue;
                            //ushort maxDepth = 2700;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            // maxDepth = depthFrame.DepthMaxReliableDistance;

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            depthFrameProcessed = true;
                        }
                    }
                }
            }

            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();
            }
        }
        void _colorReader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (var colorFrame = e.FrameReference.AcquireFrame())
             
            {
                if (colorFrame != null)
                   
                {
           
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                       _bmp.Lock();

                        // verify data and write the new color frame data to the display bitmap
                      if ((colorFrameDescription.Width == _bmp.PixelWidth) && (colorFrameDescription.Height == _bmp.PixelHeight))
                       {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                _bmp.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                               ColorImageFormat.Bgra);

                            _bmp.AddDirtyRect(new Int32Rect(0, 0, _bmp.PixelWidth, _bmp.PixelHeight));

                       }
                            
                        _bmp.Unlock();
                    }
                }
            }
        }

        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
        }

        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
        }
        #endregion

        private void Start_Measure(object sender, RoutedEventArgs e)
        {
            rotation.Content = "90";
            timerReadMes.Start();
          //  SliderLines.Start();
          //  rgblive.Visibility = System.Windows.Visibility.Hidden;
            
        }

        private void DepthImage_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {

            Point p = e.GetPosition(rgb);
            if (this._depthData != null && this._depthData.Length > 0)
            {

                var x = Math.Floor(p.X * _depthWidth / this.rgb.ActualWidth);
                var y = Math.Floor(p.Y * _depthHeight / this.rgb.ActualHeight);
                var pixel = x + (y * _depthWidth);
                if (pixel > _depthData.Length)
                    return;
                var depth = _depthData[(int)pixel];
                int depthInches = (int)(depth * 0.0393700787);
                int depthFt = depthInches / 12;
                depthInches = depthInches % 12;
                PixelDepth.Text = string.Format("x:{0}  y:{1}  z:{2}mm ~ {3}'{4}\"", x, y, depth, depthFt, depthInches);
                {

                }
            }
        }
        #region MesureControl
        private void timerReadMes_Tick(object sender, EventArgs e)
        {

            if (this._depthData != null && this._depthData.Length > 0)
                counter = counter + 1;
            {

                int[] RedLine;
                RedLine = new int[Convert.ToInt32(sliderRed_R.Value)];
                int[] RedpixelIndexcall;
                RedpixelIndexcall = new int[512];
             
                Reddepthcall = new int[512];

                int[] BlueLine;
                BlueLine = new int[Convert.ToInt32(sliderBlue_R.Value)];
                int[] BluepixelIndexcall;
                BluepixelIndexcall = new int[512];
                
                Bluedepthcall = new int[512];

                int[] GreenLine;
                GreenLine = new int[Convert.ToInt32(sliderGreen_B.Value)];
                int[] GreenpixelIndexcall;
                GreenpixelIndexcall = new int[512];
               
                Greendepthcall = new int[512];

                int[] YellowLine;
                YellowLine = new int[Convert.ToInt32(sliderYellow_B.Value)];
                int[] YellowpixelIndexcall;
                YellowpixelIndexcall = new int[512];
              
                Yellowdepthcall = new int[512];

             /*   int[] array124;
                array124 = new int[123];
                int[] array247;
                array247 = new int[123];
                int[] array370;
                array370 = new int[123];
                int[] array493;
                array493 = new int[5];

                int[] Bluearray124;
                Bluearray124 = new int[123];
                int[] Bluearray247;
                Bluearray247 = new int[123];
                int[] Bluearray370;
                Bluearray370 = new int[123];
                int[] Bluearray493;
                Bluearray493 = new int[5];

                int[] Greenarray124;
                Greenarray124 = new int[123];
                int[] Greenarray247;
                Greenarray247 = new int[123];
                int[] Greenarray370;
                Greenarray370 = new int[123];
                int[] Greenarray493;
                Greenarray493 = new int[5];

                int[] Yellowarray124;
                Yellowarray124 = new int[123];
                int[] Yellowarray247;
                Yellowarray247 = new int[123];
                int[] Yellowarray370;
                Yellowarray370 = new int[123];
                int[] Yellowarray493;
                Yellowarray493 = new int[5];*/

                int[] RedmatchedItems;
                int[] BluematchedItems;
                int[] GreenmatchedItems;
                int[] YellowmatchedItems;

                int lBound = Convert.ToInt32(Low.Text);
                int uBound = Convert.ToInt32(HIGH1.Text);

                {
                    // for (int i = xfactorColor.Length; i-- > 0; )
                    //     xfactorColor[i] = i + 1;
                    for (int r = RedLine.Length; r-- > 0; )
                        RedLine[r] = r + 1;

                    for (int b = BlueLine.Length; b-- > 0; )
                        BlueLine[b] = b + 1;

                    for (int g = 0; g != (GreenLine.Length); g++)
                        GreenLine[g] = g + 1;

                    for (int y = 0; y != (YellowLine.Length); y++)
                        YellowLine[y] = y + 1;
                }
                int RedY = Convert.ToInt32(sliderYRed.Value);
                int Redwidth = Convert.ToInt32(_depthWidth);
                for (int r = Convert.ToInt32(sliderRed_L.Value); r != (RedLine.Length); r++)
                    RedpixelIndexcall[r] = (int)(RedLine[r] + ((int)RedY * Redwidth));

                int BlueY = Convert.ToInt32(sliderYBlue.Value);
                int Bluewidth = Convert.ToInt32(_depthWidth);
                for (int b = Convert.ToInt32(sliderBlue_L.Value); b != (BlueLine.Length); b++)
                    BluepixelIndexcall[b] = (int)(BlueLine[b] + ((int)BlueY * Bluewidth));

                int GreenX = Convert.ToInt32(sliderXGrn.Value);
                int Greenwidth = Convert.ToInt32(_depthWidth);
                for (int g = Convert.ToInt32(sliderGreen_T.Value); g != (GreenLine.Length); g++)
                    GreenpixelIndexcall[g] = (int)(GreenLine[g] + ((int)GreenX * Greenwidth));

                int YellowX = Convert.ToInt32(sliderXYellow.Value);
                int Yellowidth = Convert.ToInt32(_depthWidth);
                for (int y = Convert.ToInt32(sliderYellow_T.Value); y != (YellowLine.Length); y++)
                    YellowpixelIndexcall[y] = (int)(YellowLine[y] + ((int)YellowX * Yellowidth));


                {
                    //  for (int i = 0; i != (pixelIndexcall.Length); i++)
                    for (int r = RedpixelIndexcall.Length; r-- > 0; )
                        Reddepthcall[r] = _depthData[RedpixelIndexcall[r]];
                    RedmatchedItems = Array.FindAll(Reddepthcall, x =>
                    x >= lBound && x <= uBound);
                    for (int ctr = 0; ctr < RedmatchedItems.Length; ctr++) ;
                    int Redcount = RedmatchedItems.Length;
                    Double Redsum = Convert.ToDouble(Settings1.Default.RedPixelSet);
                    RedCount.Content = Convert.ToString(Redcount);
                    RedSum.Content = Convert.ToString(Redcount * Redsum);
                    Red_Dispaly_Result = Convert.ToString(Redcount * Redsum);
                    //  CookieJar.WriteData("CookieJar_Int_Array[1]", Convert.ToString(Redcount * Redsum));


                    for (int b = BluepixelIndexcall.Length; b-- > 0; )
                        Bluedepthcall[b] = _depthData[BluepixelIndexcall[b]];
                    BluematchedItems = Array.FindAll(Bluedepthcall, x =>
                    x >= lBound && x <= uBound);
                    for (int ctr = 0; ctr < BluematchedItems.Length; ctr++) ;
                    int Bluecount = BluematchedItems.Length;
                    Double Bluesum = Convert.ToDouble(Settings1.Default.BluePixelSet);
                    BlueCount.Content = Convert.ToString(Bluecount);
                    BlueSum.Content = Convert.ToString(Bluecount * Bluesum);
                    Blue_Dispaly_Result = Convert.ToString(Bluecount * Bluesum);
                    //   CookieJar.WriteData("CookieJar_Int_Array[2]", Convert.ToString(Bluecount * Bluesum));


                    for (int b = GreenpixelIndexcall.Length; b-- > 0; )
                        Greendepthcall[b] = _depthData[GreenpixelIndexcall[b]];
                    GreenmatchedItems = Array.FindAll(Greendepthcall, x =>
                    x >= lBound && x <= uBound);
                    for (int ctr = 0; ctr < GreenmatchedItems.Length; ctr++) ;
                    int Greencount = GreenmatchedItems.Length;
                    Double Greensum = Convert.ToDouble(Settings1.Default.GreenPixelSet);
                    GreenCount.Content = Convert.ToString(Greencount);
                    GreenSum.Content = Convert.ToString(Greencount * Greensum);
                    Green_Dispaly_Result = Convert.ToString(Greencount * Greensum);
                    //  CookieJar.WriteData("CookieJar_Int_Array[3]", Convert.ToString(Greencount * Greensum));

                    for (int b = YellowpixelIndexcall.Length; b-- > 0; )
                        Yellowdepthcall[b] = _depthData[YellowpixelIndexcall[b]];
                    YellowmatchedItems = Array.FindAll(Yellowdepthcall, x =>
                    x >= lBound && x <= uBound);
                    for (int ctr = 0; ctr < YellowmatchedItems.Length; ctr++) ;
                    int Yellowcount = YellowmatchedItems.Length;
                    Double Yellowsum = Convert.ToDouble(Settings1.Default.YellowPixelSet);
                    YellowCount.Content = Convert.ToString(Yellowcount);
                    YellowSum.Content = Convert.ToString(Yellowcount * Yellowsum);
                    Yellow_Dispaly_Result = Convert.ToString(Yellowcount * Yellowsum);
                    //  CookieJar.WriteData("CookieJar_Int_Array[4]", Convert.ToString(Yellowcount * Yellowsum));
                }

                {
                 /*   if (counter == 1)
                    {
                        Array.Copy(Bluedepthcall, 124, Bluearray124, 0, 123);
                        Array.Copy(Bluedepthcall, 247, Bluearray247, 0, 123);
                        Array.Copy(Bluedepthcall, 370, Bluearray370, 0, 123);
                        CookieJar.WriteData("Pixel_0[0]", 123, Bluedepthcall);
                        CookieJar.WriteData("Pixel_0[123]", 123, Bluearray124);
                        CookieJar.WriteData("Pixel_0[246]", 123, Bluearray247);
                        CookieJar.WriteData("Pixel_0[370]", 123, Bluearray370);
                    }

                    else if (counter == 2)
                    { 
                        Array.Copy(Reddepthcall, 124, array124, 0, 123);
                       Array.Copy(Reddepthcall, 247, array247, 0, 123);
                       Array.Copy(Reddepthcall, 370, array370, 0, 123);
                       CookieJar.WriteData("Pixel_1[0]", 123, Reddepthcall);
                       CookieJar.WriteData("Pixel_1[123]", 123, array124);
                       CookieJar.WriteData("Pixel_1[246]", 123, array247);
                       CookieJar.WriteData("Pixel_1[370]", 123, array370);
                    }

                    else if (counter == 3)
                    {
                        Array.Copy(Greendepthcall, 124, Greenarray124, 0, 123);
                        Array.Copy(Greendepthcall, 247, Greenarray247, 0, 123);
                        Array.Copy(Greendepthcall, 370, Greenarray370, 0, 123);
                        CookieJar.WriteData("Pixel_2[0]", 123, Greendepthcall);
                        CookieJar.WriteData("Pixel_2[123]", 123, Greenarray124);
                        CookieJar.WriteData("Pixel_2[246]", 123, Greenarray247);
                        CookieJar.WriteData("Pixel_2[370]", 123, Greenarray370);
                    }

                    else if (counter == 4)
                    {
                        Array.Copy(Yellowdepthcall, 124, Yellowarray124, 0, 123);
                        Array.Copy(Yellowdepthcall, 247, Yellowarray247, 0, 123);
                        Array.Copy(Yellowdepthcall, 370, Yellowarray370, 0, 123);
                        CookieJar.WriteData("Pixel_3[0]", 123, Yellowdepthcall);
                        CookieJar.WriteData("Pixel_3[123]", 123, Yellowarray124);
                        CookieJar.WriteData("Pixel_3[246]", 123, Yellowarray247);
                        CookieJar.WriteData("Pixel_3[370]", 123, Yellowarray370);
                    }*/
                }
                {
                List<TodoItem> items = new List<TodoItem>();
                if (radioButton1.IsChecked == true)
                    for (int r = Reddepthcall.Length; r-- > 0; r -= 0)
                      //  items.Add(new TodoItem() { Completion = Convert.ToInt32(Reddepthcall[r] - 2200) * 6 });
                      items.Add(new TodoItem() { Completion = Convert.ToInt32(Reddepthcall[r]) });
                //      for (int r = RedmatchedItems.Length; r-- > 0; )
                //     items.Add(new TodoItem() { Completion = Convert.ToInt32(RedmatchedItems[r] * 10) });

                else if (radioButton2.IsChecked == true)
                    for (int b = Bluedepthcall.Length; b-- > 0; b -= 0)
                     //   items.Add(new TodoItem() { Completion = Convert.ToInt32(Bluedepthcall[b] - 2200) * 6 });
                      items.Add(new TodoItem() { Completion = Convert.ToInt32(Bluedepthcall[b]) });
                //     for (int b = BluematchedItems.Length; b-- > 0; )
               //   items.Add(new TodoItem() { Completion = Convert.ToInt32(BluematchedItems[b] * 10) });

                else if (radioButton3.IsChecked == true)
                    for (int g = Greendepthcall.Length; g-- > 0; g -= 0)
                    //    items.Add(new TodoItem() { Completion = Convert.ToInt32(Greendepthcall[g] - 2200) * 6 });
                     items.Add(new TodoItem() { Completion = Convert.ToInt32(Greendepthcall[g]) });
                //  for (int g = GreenmatchedItems.Length; g-- > 0; )
                //items.Add(new TodoItem() { Completion = Convert.ToInt32(GreenmatchedItems[g] * 10) });

                else if (radioButton4.IsChecked == true)
                    for (int y = Yellowdepthcall.Length; y-- > 0; y -= 0)
                     //   items.Add(new TodoItem() { Completion = Convert.ToInt32(Yellowdepthcall[y] - 2200) * 6 });
                    items.Add(new TodoItem() { Completion = Convert.ToInt32(Yellowdepthcall[y])  });
                // for (int y = YellowmatchedItems.Length; y-- > 0; )
                //  items.Add(new TodoItem() { Completion = Convert.ToInt32(YellowmatchedItems[y] * 10) });

                icTodoList.ItemsSource = items;
                count.Text = Convert.ToString(counter);
                
                }
                if (counter >= 4)
                {
                    counter = 0;
                }
            }
        }
        public class TodoItem
        {
            public int Completion { get; set; }
        }

        #endregion

        #region AdvancedPLC_Comms
        private void timerReadPLC_Tick(object sender, EventArgs e)
        {
            // string[] Display;
            // Display =  new string[5];
            int[] array124;
            array124 = new int[123];
            int[] array247;
            array247 = new int[123];
            int[] array370;
            array370 = new int[123];
            int[] array493;
            array493 = new int[5];

            int[] Bluearray124;
            Bluearray124 = new int[123];
            int[] Bluearray247;
            Bluearray247 = new int[123];
            int[] Bluearray370;
            Bluearray370 = new int[123];
            int[] Bluearray493;
            Bluearray493 = new int[5];

            int[] Greenarray124;
            Greenarray124 = new int[123];
            int[] Greenarray247;
            Greenarray247 = new int[123];
            int[] Greenarray370;
            Greenarray370 = new int[123];
            int[] Greenarray493;
            Greenarray493 = new int[5];

            int[] Yellowarray124;
            Yellowarray124 = new int[123];
            int[] Yellowarray247;
            Yellowarray247 = new int[123];
            int[] Yellowarray370;
            Yellowarray370 = new int[123];
            int[] Yellowarray493;
            Yellowarray493 = new int[5];
            counter1 = counter1 + 1;
            count1.Text = Convert.ToString(counter1);

            try
            {
                // for (int i = 0; i != (Display.Length); i++)
                //      Display[i] = CookieJar.ReadAny("Cookie_Jar_Real_Array["+ (i) +"]");
                //  Display1.Text = Display[1] + "  in";
                //  Display2.Text = Display[2] + "  in";
                //  Display3.Text = Display[3] + "  in";
                //  Display4.Text = Display[4] + "  in";

                if (counter1 == 1)
                {
                    Array.Copy(Bluedepthcall, 124, Bluearray124, 0, 123);
                    Array.Copy(Bluedepthcall, 247, Bluearray247, 0, 123);
                    Array.Copy(Bluedepthcall, 370, Bluearray370, 0, 123);
                    CookieJar.WriteData("Pixel_0[0]", 123, Bluedepthcall);
                    CookieJar.WriteData("Pixel_0[123]", 123, Bluearray124);
                    CookieJar.WriteData("Pixel_0[246]", 123, Bluearray247);
                    CookieJar.WriteData("Pixel_0[370]", 123, Bluearray370);
                }

                else if (counter1 == 2)
                {
                    Array.Copy(Reddepthcall, 124, array124, 0, 123);
                    Array.Copy(Reddepthcall, 247, array247, 0, 123);
                    Array.Copy(Reddepthcall, 370, array370, 0, 123);
                    CookieJar.WriteData("Pixel_1[0]", 123, Reddepthcall);
                    CookieJar.WriteData("Pixel_1[123]", 123, array124);
                    CookieJar.WriteData("Pixel_1[246]", 123, array247);
                    CookieJar.WriteData("Pixel_1[370]", 123, array370);
                }

                else if (counter1 == 3)
                {
                    Array.Copy(Greendepthcall, 124, Greenarray124, 0, 123);
                    Array.Copy(Greendepthcall, 247, Greenarray247, 0, 123);
                    Array.Copy(Greendepthcall, 370, Greenarray370, 0, 123);
                    CookieJar.WriteData("Pixel_2[0]", 123, Greendepthcall);
                    CookieJar.WriteData("Pixel_2[123]", 123, Greenarray124);
                    CookieJar.WriteData("Pixel_2[246]", 123, Greenarray247);
                    CookieJar.WriteData("Pixel_2[370]", 123, Greenarray370);
                }

                else if (counter1 == 4)
                {
                    Array.Copy(Yellowdepthcall, 124, Yellowarray124, 0, 123);
                    Array.Copy(Yellowdepthcall, 247, Yellowarray247, 0, 123);
                    Array.Copy(Yellowdepthcall, 370, Yellowarray370, 0, 123);
                    CookieJar.WriteData("Pixel_3[0]", 123, Yellowdepthcall);
                    CookieJar.WriteData("Pixel_3[123]", 123, Yellowarray124);
                    CookieJar.WriteData("Pixel_3[246]", 123, Yellowarray247);
                    CookieJar.WriteData("Pixel_3[370]", 123, Yellowarray370);
                }
                
                   

                Comms_Status_Display.Text = "Comm ok";
                Comms_Status_Display.Foreground = new SolidColorBrush(Colors.Green);
               }                        
            catch (Exception)
            {
                Comms_Status_Display.Text = "Comm Error Check connection and Restart";
                Comms_Status_Display.Foreground = new SolidColorBrush(Colors.Red);
                timerReadPLC.Stop();
            } 
         if (counter1 >= 4)
                {
                   counter1 = 0;
                }
        }
        private void Start_PLC(object sender, RoutedEventArgs e)
        {
                try
                {
                    CookieJar.WriteData("CookieJar_Bit_Array[0]", 1);
                    timerReadPLC.Start();
                    Comms_Status_Display.Text = "Comm ok";
                    Comms_Status_Display.Foreground = new SolidColorBrush(Colors.Green);
                 //   Start_Measure_Button.Visibility = Visibility.Visible;
                }
                catch (Exception)
                {
                    Comms_Status_Display.Text = "Comm Error Check I.P Address or Cable";
                    Comms_Status_Display.Foreground = new SolidColorBrush(Colors.Red);
                }
                   
        }
        #endregion
     
        #region  GraphControls  
        private void radioButton1_Checked(object sender, RoutedEventArgs e)
        {
            GraphColor.Text = "Red";
            GraphColor.Foreground = new SolidColorBrush(Colors.Red);
        }

        private void radioButton2_Checked(object sender, RoutedEventArgs e)
        {
            GraphColor.Text = "Blue";
            GraphColor.Foreground = new SolidColorBrush(Colors.Blue);
        }

        private void radioButton3_Checked(object sender, RoutedEventArgs e)
        {
            GraphColor.Text = "Green";
            GraphColor.Foreground = new SolidColorBrush(Colors.Green);
        }

        private void radioButton4_Checked(object sender, RoutedEventArgs e)
        {
            GraphColor.Text = "Yellow";
            GraphColor.Foreground = new SolidColorBrush(Colors.Yellow);
        }

      
        # endregion

        #region SliderControl
        void Sliderlines_Tick(object sender, EventArgs e)
        {
            myCanvas.Children.Clear();
            Graph_Canvas.Children.Clear();
            {
                Line line = new Line();
                Thickness thickness = new Thickness(331, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 2;
                line.Stroke = System.Windows.Media.Brushes.Red;
                line.X1 = (Convert.ToInt32(sliderRed_R.Value) * 1.24) - 77;
                line.X2 = (Convert.ToInt32(sliderRed_L.Value) * 1.24) - 77;
                line.Y1 = (Convert.ToInt32(sliderYRed.Value) * 1.24);
                line.Y2 = (Convert.ToInt32(sliderYRed.Value) * 1.24);
                myCanvas.Children.Add(line);
            }

            {
                Line line = new Line();
                Thickness thickness = new Thickness(331, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 2;
                line.Stroke = System.Windows.Media.Brushes.Blue;
                line.X1 = (Convert.ToInt32(sliderBlue_R.Value) * 1.24) - 77;
                line.X2 = (Convert.ToInt32(sliderBlue_L.Value) * 1.24) - 77;
                line.Y1 = (Convert.ToInt32(sliderYBlue.Value) * 1.24);
                line.Y2 = (Convert.ToInt32(sliderYBlue.Value) * 1.24);
                myCanvas.Children.Add(line);
            }

            {
                Line line = new Line();
                Thickness thickness = new Thickness(331, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 3;
                line.Stroke = System.Windows.Media.Brushes.Green;
                line.X1 = (Convert.ToInt32(sliderGreen_T.Value) * 1.24) - 77;
                line.X2 = (Convert.ToInt32(sliderGreen_B.Value) * 1.24) - 77;
                line.Y1 = (Convert.ToInt32(sliderXGrn.Value) * 1.24);
                line.Y2 = (Convert.ToInt32(sliderXGrn.Value) * 1.24);
                myCanvas.Children.Add(line);

            }

            {
                Line line = new Line();
                Thickness thickness = new Thickness(331, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 2;
                line.Stroke = System.Windows.Media.Brushes.Yellow;
                line.X1 = (Convert.ToInt32(sliderYellow_T.Value) * 1.24) - 77;
                line.X2 = (Convert.ToInt32(sliderYellow_B.Value) * 1.24) - 77;
                line.Y1 = (Convert.ToInt32(sliderXYellow.Value) * 1.24);
                line.Y2 = (Convert.ToInt32(sliderXYellow.Value) * 1.24);
                myCanvas.Children.Add(line);

            }

            {
                Line line = new Line();
                Thickness thickness = new Thickness(-250, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 1;
                line.Stroke = System.Windows.Media.Brushes.Black;
                line.X1 = -50;
                line.X2 = 1100;
                line.Y1 = (Convert.ToInt32(sliderGraph_low.Value) / 12) ;
                line.Y2 = (Convert.ToInt32(sliderGraph_low.Value) / 12) ;
                Graph_Canvas.Children.Add(line);
            }

            {
                Line line = new Line();
                Thickness thickness = new Thickness(-250, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 1;
                line.Stroke = System.Windows.Media.Brushes.Black;
                line.X1 = -50;
                line.X2 = 1100;
                line.Y1 = (Convert.ToInt32(sliderGraph_High.Value) / 4) - 200;
                line.Y2 = (Convert.ToInt32(sliderGraph_High.Value) / 4) - 200;
                Graph_Canvas.Children.Add(line);
            }

            {
                SliderLines.Stop();
            }
        }

        private void sliderGraph_High_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
                SliderLines.Start();
               HIGH1.Text = Convert.ToString(sliderGraph_High.Value);   
        }
        
        private void sliderGraph_lowValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
           SliderLines.Start();
           Low.Text = Convert.ToString(sliderGraph_low.Value);         
        }
     
        private void sliderRed_L_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }
        private void sliderYRed_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
            this.radioButton1.IsChecked = true;                     
        }
              
        private void sliderRed_R_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderXGrn_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
            this.radioButton3.IsChecked = true;                
        }
        
        private void sliderGreen_B_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderGreen_T_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderXYellow_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
            this.radioButton4.IsChecked = true;             
        }
        
        private void sliderYellow_T_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderYellow_B_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderBlue_R_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderYBlue_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
            this.radioButton2.IsChecked = true;         
        }
        
        private void sliderBlue_L_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        # endregion

        #region Store
        private void Save_IP_Address(object sender, RoutedEventArgs e)
        {
            // Update the value.
            Settings1.Default.IP_Address = IP_Address.Text;
            // Save the config file.
            Settings1.Default.Save();
        }

        private void Save_Button_Click(object sender, RoutedEventArgs e)
        {
            if (string.IsNullOrWhiteSpace(this.RedPixelSet.Text))
            {
                MessageBox.Show("TextBox is empty");
            }
            else
                // Update the value.
                Settings1.Default.RedPixelSet = RedPixelSet.Text;
            Settings1.Default.BluePixelSet = BluePixelSet.Text;
            Settings1.Default.GreenPixelSet = GreenPixelSet.Text;
            Settings1.Default.YellowPixelSet = YellowPixelSet.Text;
            // Save the config file.
            Settings1.Default.Save();
        }

        private void Save_Button_Red_Click(object sender, RoutedEventArgs e)
        {
            {
            if (string.IsNullOrWhiteSpace(this.RedPixelSet.Text))
            {
                MessageBox.Show("TextBox is empty");
            }
            else
                // Update the value.
                Settings1.Default.RedPixelSet = RedPixelSet.Text;
            // Save the config file.
            Settings1.Default.Save();
            }
            {
                MessageBox.Show("Pixel Dia. has been saved.");
            }
        }

        private void Save_Button_Blue_Click(object sender, RoutedEventArgs e)
        {
            {
                if (string.IsNullOrWhiteSpace(this.BluePixelSet.Text))
                {
                    MessageBox.Show("TextBox is empty");
                }
                else
                    // Update the value.
                Settings1.Default.BluePixelSet = BluePixelSet.Text;
                // Save the config file.
                Settings1.Default.Save();
            }
            {
                MessageBox.Show("Pixel Dia. has been saved.");
            }
        }


        private void Save_Button_Green_Click(object sender, RoutedEventArgs e)
        {
          {
            if (string.IsNullOrWhiteSpace(this.GreenPixelSet.Text))
            {
                MessageBox.Show("TextBox is empty");
            }
            else
                // Update the value.
            Settings1.Default.GreenPixelSet = GreenPixelSet.Text;
            // Save the config file.
            Settings1.Default.Save();
           }
       
           {
             MessageBox.Show("Pixel Dia. has been saved.");
           }
        }

        private void Save_Button_Yellow_Click(object sender, RoutedEventArgs e)
        {
            {
                if (string.IsNullOrWhiteSpace(this.YellowPixelSet.Text))
                {
                    MessageBox.Show("TextBox is empty");
                }
                else
                    // Update the value.
                Settings1.Default.YellowPixelSet = YellowPixelSet.Text;
                // Save the config file.
                Settings1.Default.Save();
            }
            {
                MessageBox.Show("Pixel Dia. has been saved.");
            }
        }

        private void RedPixelSet_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (System.Text.RegularExpressions.Regex.IsMatch(RedPixelSet.Text, "[^.0-9]"))
            {
                MessageBox.Show("Please enter only numbers.");
               RedPixelSet.Text = RedPixelSet.Text.Remove(RedPixelSet.Text.Length - 1);
            }
        }

        private void BluePixelSet_TextChanged(object sender, TextChangedEventArgs e)
        {
           if (System.Text.RegularExpressions.Regex.IsMatch(BluePixelSet.Text, "[^.0-9]"))
            {
                MessageBox.Show("Please enter only numbers.");
                BluePixelSet.Text = BluePixelSet.Text.Remove(BluePixelSet.Text.Length - 1);
            }
        }

        private void GreenPixelSet_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (System.Text.RegularExpressions.Regex.IsMatch(GreenPixelSet.Text, "[^.0-9]"))
            {
                MessageBox.Show("Please enter only numbers.");
                GreenPixelSet.Text = GreenPixelSet.Text.Remove(GreenPixelSet.Text.Length - 1);
            }
        }

        private void YellowPixelSet_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (System.Text.RegularExpressions.Regex.IsMatch(YellowPixelSet.Text, "[^.0-9]"))
            {
                MessageBox.Show("Please enter only numbers.");
                YellowPixelSet.Text = YellowPixelSet.Text.Remove(YellowPixelSet.Text.Length - 1);
            }
        }

        private void IP_Address_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (System.Text.RegularExpressions.Regex.IsMatch(IP_Address.Text, "[^.0-9]"))
            {
                MessageBox.Show("Please enter only numbers.");
                IP_Address.Text = IP_Address.Text.Remove(IP_Address.Text.Length - 1);
            }
           }      
         }
      }
# endregion