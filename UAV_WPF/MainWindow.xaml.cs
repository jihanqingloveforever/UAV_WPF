using GMap.NET;
using GMap.NET.MapProviders;
using GMap.NET.WindowsPresentation;
using MAVLink;
using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Data;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using UAV_WPF.Tools;

namespace UAV_WPF
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    ///  //解析数据帧委托

    public delegate void AnalyzeMavLinkFrame(byte[] mavLinkFrame_2);
    public partial class MainWindow : Window
    {
        #region 测试定义的一些变量
        StringBuilder sba = new StringBuilder();
        StringBuilder sbb = new StringBuilder();
        Point point;
        List<Point> list;
        #endregion

        public Type type = MethodBase.GetCurrentMethod().DeclaringType;
        BitmapImage pinSrcImage;
        PointLatLng firstPoint, lastPoint;
        bool IsFirstClickForPloylay = false;
        int EditWayPoint = 1;
        MouseType mouseType = MouseType.None;
        public AccessMode accessModel = AccessMode.ServerAndCache;
        public GMapProvider gMapProvider = GMapProviders.GoogleChinaMap;
        List<WayPointInfo> wayPointInfoList = new List<WayPointInfo>();
        List<LocationWayPoint> cmdlist = new List<LocationWayPoint>();
        SerialPort sp = new SerialPort();
        int bufferNum;
        List<byte> mavDataList = new List<byte>();
        byte[] mavLinkFrame = null;
        MAVLink.MavLink.MavLinkFrame_2 dataFrame_2;
        public AnalyzeMavLinkFrame analyzeMavLinkFrame;
        byte[] oldHeartPack = new byte[17];
        MavStatus mavStatus = new MavStatus();
        bool isSuo = true;
        bool isOpen = true;//第一次打开 
        private List<GMapMarker> RouiteMarkList = new List<GMapMarker>();//轨迹上的标记点
        public PointLatLngAlt latlngalt;
        public List<PointLatLngAlt> pointlatlngaltList = new List<PointLatLngAlt>();//标记点集合--经度维度高度信息
        /// <summary>
        /// 任务回应 让其初始化的时候 result=255
        /// </summary>
        MavLink.mavlink_Mission_Ack mission_Ack = new MavLink.mavlink_Mission_Ack();
        /// <summary>
        /// 命令应答 
        /// </summary>
        MavLink.mavlink_Command_Ack command_Ack = new MavLink.mavlink_Command_Ack();
        /// <summary>
        /// 任务计数 解析44号消息包让其mission_count.count = 1000
        /// </summary>
        MAVLink.MavLink.mavlink_Mission_Count mission_count = new MavLink.mavlink_Mission_Count();
        /// <summary>
        /// 任务下载请求 
        /// </summary>
        MavLink.mavlink_mission_request mission_request = new MavLink.mavlink_mission_request();
        /// <summary>
        /// 任务项 解析39号消息包 mission_iteam.seq=255
        /// </summary>
        MAVLink.MavLink.mavlink_Mission_Item mission_iteam = new MavLink.mavlink_Mission_Item();


        /// <summary>
        /// 画多边形的集合
        /// </summary>
        List<PointLatLng> pointLatLngs = new List<PointLatLng>();
        int zoom = 5;
        public MainWindow()
        {
            InitializeComponent();
            MyMapControl.MouseMove += MyMapControl_MouseMove;
            MyMapControl.MouseDoubleClick += MyMapControl_MouseDoubleClick;
            ckAddMarker.Click += CkAddMarker_Click;
            command_Ack.result = 255;
            mission_Ack.type = 255;
            mission_count.count = 255;
            bandingFlyInfo();
        }
        /// <summary>
        /// 绑定飞行控制事件
        /// </summary>
        private void bandingFlyInfo()
        {
            flyinfo.btnSetMode.Click += BtnSetMode_Click;
            flyinfo.btnSuo.Click += BtnSuo_Click;
            flyinfo.btnSetSpeed.Click += BtnSetSpeed_Click;
            flyinfo.btnHeight.Click += BtnHeight_Click;
            flyinfo.btnInisAlt.Click += BtnInisAlt_Click;
            flyinfo.btnFly.Click += BtnFly_Click;
            flyinfo.btnReset.Click += BtnReset_Click;
            flyinfo.btnHuiFuMission.Click += BtnHuiFuMission_Click;
        }
        /// <summary>
        /// 地图状态改变
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void GMapChange(object sender, SelectionChangedEventArgs e)
        {
            zoom = Convert.ToInt32(cmbGMapZoom.SelectedValue);
            gMapProvider = (GMapProvider)cmbGMapChoise.SelectedValue;
            accessModel = (AccessMode)cmbGMapModel.SelectedValue;
            LoadMap();
        }


        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            BindingData();
            cmbGMapChoise.SelectionChanged += GMapChange;
            cmbGMapZoom.SelectionChanged += GMapChange;
            cmbGMapModel.SelectionChanged += GMapChange;
            MyMapControl.Width = mapCanvas.ActualWidth;
            MyMapControl.Height = mapCanvas.ActualHeight;
            SetEditWayButton();
            LoadMap();
        }
        /// <summary>
        /// 设置编辑航点的属性
        /// </summary>
        private void SetEditWayButton()
        {
            btnReadWayPoint.IsEnabled = false;
            btnWriteWayPoint.IsEnabled = false;
            btnClearWayPoint.IsEnabled = false;
            btnSaveWayPoint.IsEnabled = false;
            btnOpenWayPoint.IsEnabled = false;
        }
        /// <summary>
        /// 加载地图
        /// </summary>
        private void LoadMap()
        {
            try
            {
                System.Net.IPHostEntry e = System.Net.Dns.GetHostEntry("ditu.google.cn");
            }
            catch (Exception ex)
            {
                MyMapControl.Manager.Mode = accessModel;// GMap.NET.AccessMode.ServerAndCache;

                MessageBox.Show("No internet connection avaible, going to CacheOnly mode. 异常信息" + ex.ToString(), "GMap.NET Demo", MessageBoxButton.OK, MessageBoxImage.Warning);
            }
            MyMapControl.MapProvider = gMapProvider; //google china 地图
            MyMapControl.MinZoom = 1;  //最小缩放
            MyMapControl.MaxZoom = 18; //最大缩放
            MyMapControl.Zoom = Convert.ToInt16(cmbGMapZoom.SelectedValue);     //当前缩放
            MyMapControl.ShowCenter = true; //不显示中心十字点
            MyMapControl.DragButton = MouseButton.Left; //左键拖拽地图
            MyMapControl.Position = new PointLatLng(34.75734, 113.635); //地图中心位置：南京

        }
        /// <summary>
        /// 绑定一些数据元
        /// </summary>
        private void BindingData()
        {
            Global.mavStatus = this.mavStatus;
            cmbGMapChoise.ItemsSource = GMapProviders.List;
            cmbGMapChoise.SelectedValue = GMapProviders.GoogleChinaMap; //google china 地图
            cmbGMapModel.Items.Add(AccessMode.CacheOnly);
            cmbGMapModel.Items.Add(AccessMode.ServerAndCache);
            cmbGMapModel.Items.Add(AccessMode.ServerOnly);
            cmbGMapModel.SelectedValue = AccessMode.ServerAndCache;
            List<int> zoomList = new List<int>();
            for (int i = 1; i <= 18; i++)
                zoomList.Add(i);
            cmbGMapZoom.ItemsSource = zoomList;
            cmbGMapZoom.SelectedIndex = 5;
            int[] baudRates = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 111100, 115200, 500000, 921600, 1500000 };
            cmbBaud.ItemsSource = baudRates;
            cmbBaud.SelectedIndex = 8;
            cmbPortName.ItemsSource = SerialPort.GetPortNames();
            cmbPortName.SelectedIndex = 0;
            cmbPortName.IsReadOnly = true;
        }

        /// <summary>
        /// 重新画地图
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void DrawMarker(object sender, MouseButtonEventArgs e)
        {
            Point pt = e.GetPosition(MyMapControl);
            PointLatLng point = MyMapControl.FromLocalToLatLng((int)pt.X, (int)pt.Y);
            AddMaker(point);
        }
        /// <summary>
        /// 添加标记
        /// </summary>
        /// <param name="pt"></param>
        public void AddMaker(PointLatLng pt)
        {
            GMapMarker marker = new GMapMarker(pt);
            marker.Shape = CreatePinImage(marker);
            this.MyMapControl.Markers.Add(marker);
        }
        /// <summary>
        /// 创建标记的图标
        /// </summary>
        /// <param name="marker"></param>
        /// <returns></returns>
        private Image CreatePinImage(GMapMarker marker)
        {
            Image image = new Image();
            image.Tag = marker;
            image.Width = 32;
            image.Height = 32;
            image.Name = "MarkerTag";
            image.MouseEnter += Image_MouseEnter;
            if (pinSrcImage == null)
            {
                pinSrcImage = new BitmapImage(new Uri(AppDomain.CurrentDomain.BaseDirectory + "red-dot.png", UriKind.Absolute));
                pinSrcImage.Freeze();
            }
            image.Source = pinSrcImage;
            marker.Offset = new Point(-image.Width / 2, -image.Height);
            return image;
        }
        /// <summary>
        /// 当移动到标记时显示的信息
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Image_MouseEnter(object sender, MouseEventArgs e)
        {
            Image image = (Image)sender;
            GMapMarker marker = (GMapMarker)image.Tag;
            image.ToolTip = marker.Position.Lat.ToString() + Environment.NewLine + marker.Position.Lng.ToString();
        }

        /// <summary>
        /// 地图双击事件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MyMapControl_MouseDoubleClick(object sender, MouseButtonEventArgs e)
        {
            if (mouseType == MouseType.None)
                return;
            else if (mouseType == MouseType.DrawMarker)
            {
                DrawMarker(sender, e);
            }
            else if (mouseType == MouseType.DrawPolygon)
            {
                Point pt = e.GetPosition(MyMapControl);
                PointLatLng point = MyMapControl.FromLocalToLatLng((int)pt.X, (int)pt.Y);
                if (IsFirstClickForPloylay == true)
                {
                    firstPoint = lastPoint = point;

                }
                if (pointLatLngs.Count >= 4)
                {
                    pointLatLngs.RemoveAt(pointLatLngs.Count - 1);

                }
                pointLatLngs.Add(point);
                pointlatlngaltList.Add(new PointLatLngAlt(point, Global.defaultAlt));
                if (pointLatLngs.Count >= 3)
                {
                    pointLatLngs.Add(lastPoint);
                }
                wayPointInfoList.Add(
                    new WayPointInfo()
                    {
                        command = "WAYPOINT",
                        lat = point.Lat,
                        lng = point.Lng,
                        heigh = 2,
                        stayTime = Convert.ToDateTime("0:0:2")
                    }
                    );
                DrawPolygon();
                WriteDVG();
                IsFirstClickForPloylay = false;
            }
        }
        /// <summary>
        /// 鼠标在地图上移动时显示的经纬度
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MyMapControl_MouseMove(object sender, MouseEventArgs e)
        {
            var point = e.GetPosition(MyMapControl);
            var latlng = MyMapControl.FromLocalToLatLng((int)point.X, (int)point.Y);
            labLatLng.Dispatcher.Invoke(new Action(() =>
            {
                labLatLng.Content = "经度：" + latlng.Lat.ToString() + Environment.NewLine + "维度：" + latlng.Lng.ToString();
            }));
        }

        /// <summary>
        /// 恢复任务
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnHuiFuMission_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                return;
            }
            if (!Helper.IsInt(flyinfo.txtWPIndex.Text.Trim()))
            {
                MessageBox.Show("请输入航点序号");
                return;
            }
            MavLink.mavlink_Mission_Set_Current set_Current = new MavLink.mavlink_Mission_Set_Current();
            set_Current.target_system = Global.sysID;
            set_Current.target_component = Global.compID;
            set_Current.seq = Convert.ToUInt16(flyinfo.txtWPIndex.Text.Trim());
            AssembleAndSendFrame((byte)MAVLink.MavLink.MAVLINK_MSG_ID.MISSION_SET_CURRENT, set_Current);
        }
        /// <summary>
        /// 重新开始任务
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnReset_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                return;
            }
            MavLink.mavlink_Mission_Set_Current set_Current = new MavLink.mavlink_Mission_Set_Current();
            set_Current.target_system = Global.sysID;
            set_Current.target_component = Global.compID;
            set_Current.seq = (ushort)0;
            AssembleAndSendFrame((byte)MAVLink.MavLink.MAVLINK_MSG_ID.MISSION_SET_CURRENT, set_Current);
        }
        /// <summary>
        /// 起飞
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnFly_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                return;
            }
            if (!Helper.IsDouble(flyinfo.txtTakeOffAlt.Text.Trim()))
            {
                MessageBox.Show("请输入目标高度");
                return;
            }
            Global.takeOffAlt = Convert.ToInt32(flyinfo.txtTakeOffAlt.Text.Trim());
            DoCommand(MavLink.MAV_CMD.TAKEOFF, 0, 0, 0, 0, 0, 0, Global.takeOffAlt);
        }
        /// <summary>
        /// 初始化高度
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnInisAlt_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                return;
            }
            DoCommand(MavLink.MAV_CMD.PREFLIGHT_CALIBRATION, 0, 0, 1, 0, 0, 0, 0);
            MessageBox.Show("初始化成功！", "消息", MessageBoxButton.OK);
        }
        /// <summary>
        /// 设置飞行高度
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnHeight_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                return;
            }
            LocationWayPoint gotohere = new LocationWayPoint();
            gotohere.alt = (float)Convert.ToDecimal(flyinfo.txtnumFlyAlt.Text);
            gotohere.id = (byte)MavLink.MAV_CMD.WAYPOINT; //航点的ID编号命ling
            SendWP(gotohere, 0, MavLink.MAV_FRAME.GLOBAL_RELATIVE_ALT, (byte)3);
            Thread.Sleep(500);
            SendWP(gotohere, 0, MavLink.MAV_FRAME.GLOBAL_RELATIVE_ALT, (byte)3);
        }
        /// <summary>
        /// 发送坐标信息
        /// </summary>
        /// <param name="loc"></param>
        /// <param name="index"></param>
        /// <param name="frame"></param>
        /// <param name="current"></param>
        /// <param name="autocontinue"></param>
        public void SendWP(LocationWayPoint loc, ushort index, MAVLink.MavLink.MAV_FRAME frame, byte current = 0, byte autocontinue = 1)
        {
            MAVLink.MavLink.mavlink_Mission_Item req = new MavLink.mavlink_Mission_Item();
            req.target_system = Global.sysID;
            req.target_component = Global.compID; // MSG_NAMES.MISSION_ITEM
            req.command = loc.id;
            req.current = current;
            req.autocontinue = autocontinue;
            req.frame = (byte)frame;
            req.y = (float)(loc.lng);
            req.x = (float)(loc.lat);
            req.z = (float)(loc.alt);
            req.param1 = loc.p1;
            req.param2 = loc.p2;
            req.param3 = loc.p3;
            req.param4 = loc.p4;
            req.seq = index;
            AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.MISSION_ITEM, req);
        }
        /// <summary>
        ///设置飞行速度 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnSetSpeed_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                return;
            }

            DoCommand(MavLink.MAV_CMD.DO_CHANGE_SPEED, 1, (float)Convert.ToDecimal(flyinfo.txtnumSetSpeed.Text), 0, 0, 0, 0, 0);
        }
        /// <summary>
        /// 锁定
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnSuo_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
                return;
            DoCommand(MAVLink.MavLink.MAV_CMD.COMPONENT_ARM_DISARM, isSuo ? 1 : 0, 21196, 0, 0, 0, 0, 0);
        }
        /// <summary>
        /// 执行命令
        /// </summary>
        /// <param name="action"></param>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        /// <param name="p4"></param>
        /// <param name="p5"></param>
        /// <param name="p6"></param>
        /// <param name="p7"></param>
        /// <returns></returns>
        public bool DoCommand(MavLink.MAV_CMD action, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
        {
            MavLink.mavlink_Command_Long command = new MavLink.mavlink_Command_Long();
            command.target_component = Global.compID;
            command.target_system = Global.sysID;
            command.command = (ushort)action;
            command.param1 = p1;
            command.param2 = p2;
            command.param3 = p3;
            command.param4 = p4;
            command.param5 = p5;
            command.param6 = p6;
            command.param7 = p7;
            AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.COMMAND_LONG, command);
            DateTime now = DateTime.Now;
            int num = 5;
            while (true)
            {
                if (command_Ack.result == 255 && now.AddMilliseconds(500) > DateTime.Now)//没有超时 但是 数据帧没有来 继续接受
                {
                    //暂时  //  Application.DoEvents();
                    continue;
                }
                else if (command_Ack.result != 255)//数据帧来了
                {
                    command_Ack.result = 255;
                    return true;
                }
                else if (now.AddMilliseconds(500) < DateTime.Now)//超时了
                {
                    if (num == 0)//5次之后 
                    {
                        return false;
                    }
                    AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.COMMAND_LONG, command);
                    num--;
                    now = DateTime.Now;
                }
            }
        }
        /// <summary>
        /// 设置飞行模式
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnSetMode_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                return;
            }
            MAVLink.MavLink.mavlink_Set_Mode mode = SetMode(flyinfo.cbMoShi.Text.Trim());
            AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.SET_MODE, mode);
            Thread.Sleep(20);
            AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.SET_MODE, mode);
        }
        /// <summary>
        /// 设置飞行模式
        /// </summary>
        /// <param name="MoShiString"></param>
        /// <returns></returns>
        public MavLink.mavlink_Set_Mode SetMode(string MoShiString)
        {
            MAVLink.MavLink.mavlink_Set_Mode set_Mode = new MAVLink.MavLink.mavlink_Set_Mode();
            switch (MoShiString)
            {
                case "自稳":
                    set_Mode.target_system = Global.compID;
                    set_Mode.custom_mode = (UInt32)0;
                    set_Mode.base_mode = (byte)1;
                    return set_Mode;
                case "定高":
                    set_Mode.target_system = Global.compID;
                    set_Mode.custom_mode = (UInt32)2;
                    set_Mode.base_mode = (byte)1;
                    return set_Mode;
                case "定点":
                    set_Mode.target_system = Global.compID;
                    set_Mode.custom_mode = (UInt32)16;
                    set_Mode.base_mode = (byte)1;
                    return set_Mode;
                case "任务":
                    set_Mode.target_system = Global.compID;
                    set_Mode.custom_mode = (UInt32)3;
                    set_Mode.base_mode = (byte)1;
                    return set_Mode;
                case "环绕":
                    set_Mode.target_system = Global.compID;
                    set_Mode.custom_mode = (UInt32)7;
                    set_Mode.base_mode = (byte)1;
                    return set_Mode;
                case "返航":
                    set_Mode.target_system = Global.compID;
                    set_Mode.custom_mode = (UInt32)6;
                    set_Mode.base_mode = (byte)1;
                    return set_Mode;
                case "降落":
                    set_Mode.target_system = Global.compID;
                    set_Mode.custom_mode = (UInt32)9;
                    set_Mode.base_mode = (byte)1;
                    return set_Mode;
                case "引导":
                    set_Mode.target_system = Global.compID;
                    set_Mode.custom_mode = (UInt32)4;
                    set_Mode.base_mode = (byte)1;
                    return set_Mode;
                default:
                    return set_Mode;
            }
        }
        /// <summary>
        ///添加标注 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void CkAddMarker_Click(object sender, RoutedEventArgs e)
        {
            if (ckAddMarker.IsChecked != null && ckAddMarker.IsChecked == true)
            {
                mouseType = MouseType.DrawMarker;
            }
            else
            {
                mouseType = MouseType.None;
            }
        }


        private void BtnEditWayPoint_MouseEnter(object sender, MouseEventArgs e)
        {
            if (string.Equals(btnEditWayPoint.Content.ToString(), "编辑航点"))
            {
                btnEditWayPoint.ToolTip = "退出编辑";
            }
            else
            {
                btnEditWayPoint.ToolTip = "编辑航点";
            }
        }

        private void BtnEditWayPoint_Click(object sender, RoutedEventArgs e)
        {
            if (EditWayPoint % 2 == 0)
            {
                btnReadWayPoint.IsEnabled = false;
                btnWriteWayPoint.IsEnabled = false;
                btnClearWayPoint.IsEnabled = false;
                btnSaveWayPoint.IsEnabled = false;
                btnOpenWayPoint.IsEnabled = false;
                btnEditWayPoint.Content = "编辑航点";
                mouseType = MouseType.None;
                IsFirstClickForPloylay = false;
            }
            else
            {
                btnReadWayPoint.IsEnabled = true;
                btnWriteWayPoint.IsEnabled = true;
                btnClearWayPoint.IsEnabled = true;
                btnSaveWayPoint.IsEnabled = true;
                btnOpenWayPoint.IsEnabled = true;
                btnEditWayPoint.Content = "退出编辑";
                mouseType = MouseType.DrawPolygon;
                IsFirstClickForPloylay = true;
            }
            EditWayPoint++;
        }
        private delegate void DelTest(List<PointLatLng> pointLatLngAlts);
        private void drwapolyne()
        {
            List<PointLatLng> pointLatLngs = PointLatLngAltToPointLatLng(pointlatlngaltList);
            try
            {
                this.Dispatcher.Invoke(()=> {
                    PointLatLng firstPointLatLng = pointLatLngs[0];
                    if (pointLatLngs.Count >= 4)
                        pointLatLngs.RemoveAt(pointLatLngs.Count - 1);
                    if (pointLatLngs.Count >= 3)
                        pointLatLngs.Add(firstPointLatLng);
                    GMapRoute route = new GMapRoute(pointLatLngs);
                    route.Shape = new Line()
                    {
                        StrokeThickness = 2,
                        Stroke = Brushes.Red
                    };
                    MyMapControl.Markers.Clear();
                    for (int i = 0; i < pointLatLngs.Count - 1; i++)
                    {
                        AddMaker(pointLatLngs[i]);
                    }
                    MyMapControl.Markers.Add(route);
                    LoadMap();
                });
            }
            catch(Exception ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }
        public void DrawPolygon(List<PointLatLngAlt> pointlatlngaltList)
        {          
            drwapolyne();
        }
        public List<PointLatLng> PointLatLngAltToPointLatLng(List<PointLatLngAlt> pointLatLngAlts)
        {
            List<PointLatLng> pointLatLngList = new List<PointLatLng>();
            foreach (var item in pointLatLngAlts)
            {
                pointLatLngList.Add(new PointLatLng()
                {
                    Lat = item.Lat,
                    Lng = item.Lng
                });
            }
            return pointLatLngList;
        }
        /// <summary>
        /// 画多边形
        /// </summary>
        public void DrawPolygon()
        { 
            if(pointLatLngs.Count == 1)
            {
                GMapMarker gMapMarker = new GMapMarker(firstPoint);
                gMapMarker.Shape = CreatePinImage(gMapMarker);
                MyMapControl.Markers.Add(gMapMarker);
            }
            else
            {
                GMap.NET.WindowsPresentation.GMapRoute route = new GMapRoute(pointLatLngs);
                route.Shape = new Line()
                {
                    StrokeThickness = 4,
                    Stroke = System.Windows.Media.Brushes.Red
                };
                MyMapControl.Markers.Clear();
                for (int i = 0; i < pointLatLngs.Count - 1; i++)
                    AddMaker(pointLatLngs[i]);
                if (pointLatLngs.Count == 2)
                    AddMaker(pointLatLngs[1]);
                MyMapControl.Markers.Add(route);
            }
            LoadMap();
        }

        private void Ok_Click(object sender, RoutedEventArgs e)
        {

        }

        private void BtnSaveWayPoint_Click(object sender, RoutedEventArgs e)
        {
            SaveFileDialog saveFile = new SaveFileDialog();
            saveFile.Filter = "文本文件(*.txt)|*.txt";
            if (saveFile.ShowDialog() == true)
            {
                string localFilePath = saveFile.FileName.ToString();//获取文件路径
                if (localFilePath != null)
                {
                    using (FileStream fs = (FileStream)saveFile.OpenFile())
                    {
                        using (StreamWriter sw = new StreamWriter(fs))
                        {
                            //foreach (var param in mavStatus.paraDic)
                            //{
                            //    sw.WriteLine(param.Key + "," + param.Value);
                            //}

                        }
                    }
                    MessageBox.Show("导出完成!");
                }
            }
        }
        #region 测试用的代码
        private void TestanalyzeMavLinkFrame(List<byte> bytes)
        {
            while (bytes.Count > 2)//
            {
                //第一个是254 且 长度够一个数据帧 (不一定是一个正确帧)
                if (bytes[0] == 254 && bytes.Count >= 8 + bytes[1])
                {
                    byte dataLength = bytes[1];
                    mavLinkFrame = new byte[8 + dataLength];
                    mavLinkFrame = bytes.GetRange(0, 8 + dataLength).ToArray();
                    //dataFrame = new MavLink.MavLinkFrame(mavLinkFrame);
                    dataFrame_2 = new MavLink.MavLinkFrame_2(mavLinkFrame);
                    ushort checksum = MavlinkCRC.crc_calculate(Helper.StructToByteArray_2(dataFrame_2), 6 + dataLength);
                    checksum = MavlinkCRC.crc_accumulate(MavlinkCRC.MAVLINK_MESSAGE_CRCS[mavLinkFrame[5]], checksum);
                    byte ck_a = (byte)(checksum & 0xFF);
                    byte ck_b = (byte)(checksum >> 8);
                    //254是数据部分的254  不是头
                    if (mavLinkFrame[mavLinkFrame.Length - 2] != ck_a || mavLinkFrame[mavLinkFrame.Length - 1] != ck_b)
                    {
                        bytes.RemoveAt(0);
                        continue;
                    }
                    else//正确的帧
                    {
                        analyzeMavLinkFrame = new AnalyzeMavLinkFrame(UpdateMavStatus);
                        analyzeMavLinkFrame(mavLinkFrame);

                        //移除这个帧
                        bytes.RemoveRange(0, 8 + dataLength);
                    }
                }
                //第一个是254 但 长度不够一个数据帧
                else if (bytes[0] == 254 && bytes.Count < 8 + bytes[1])
                {
                    //结束  继续接收数据
                    break;
                }
                //第一个不是 254
                else
                {
                    //将第一个移除
                    bytes.RemoveAt(0);
                    continue;
                }
            }
        }
        private delegate void DelTestanalyzeMavLinkFram(List<byte> bytes);
        private void testStart()
        {
            string path = @"F:\test.txt";
            string[] strs = File.ReadAllLines(path);
            int[] intArray;
            int count;
            string[] strArr;
            DelTestanalyzeMavLinkFram delTestanalyzeMavLinkFram = new DelTestanalyzeMavLinkFram(TestanalyzeMavLinkFrame);
            List<byte> vs = new List<byte>();
            foreach (string str in strs)
            {

                List<string> ss = str.Split().ToList();
                count = ss.Count - 1;
                ss.RemoveAt(count);
                strArr = ss.ToArray();
                intArray = Array.ConvertAll<string, int>(strArr, s => int.Parse(s));
                List<byte> byts = ss.ConvertAll<byte>(s => byte.Parse(s));
                mavDataList.AddRange(byts);
                while (mavDataList.Count > 2)//
                {
                    //第一个是254 且 长度够一个数据帧 (不一定是一个正确帧)
                    if (mavDataList[0] == 254 && mavDataList.Count >= 8 + mavDataList[1])
                    {
                        byte dataLength = mavDataList[1];
                        mavLinkFrame = new byte[8 + dataLength];
                        mavLinkFrame = mavDataList.GetRange(0, 8 + dataLength).ToArray();
                        dataFrame_2 = new MavLink.MavLinkFrame_2(mavLinkFrame);
                        ushort checksum = MavlinkCRC.crc_calculate(Helper.StructToByteArray_2(dataFrame_2), 6 + dataLength);
                        checksum = MavlinkCRC.crc_accumulate(MavlinkCRC.MAVLINK_MESSAGE_CRCS[mavLinkFrame[5]], checksum);
                        byte ck_a = (byte)(checksum & 0xFF);
                        byte ck_b = (byte)(checksum >> 8);
                        //254是数据部分的254  不是头
                        if (mavLinkFrame[mavLinkFrame.Length - 2] != ck_a || mavLinkFrame[mavLinkFrame.Length - 1] != ck_b)
                        {
                            mavDataList.RemoveAt(0);
                            continue;
                        }
                        else//正确的帧
                        {
                            analyzeMavLinkFrame = new AnalyzeMavLinkFrame(UpdateMavStatus);
                            analyzeMavLinkFrame(mavLinkFrame);
                            //移除这个帧
                            mavDataList.RemoveRange(0, 8 + dataLength);
                        }
                    }
                    //第一个是254 但 长度不够一个数据帧
                    else if (mavDataList[0] == 254 && mavDataList.Count < 8 + mavDataList[1])
                    {
                        //结束  继续接收数据
                        break;
                    }
                    //第一个不是 254
                    else
                    {
                        //将第一个移除
                        mavDataList.RemoveAt(0);
                        continue;
                    }
                }
            }
        }
        private void Test_Click(object sender, RoutedEventArgs e)
        {
            //int[] x = { 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103, 105, 104, 105, 106, 103 };
            //int[] y = { 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415, 412, 412, 413, 413, 415 };
            //float[] box = { 5, 5, 2, 4, 0.00001F };
            //TestHelper.cvFitEllipse2f(x, y, x.Count(), box);
            // ThreadStart threadStart = testStart;
            //  Thread thread = new Thread(threadStart);
            //  thread.Start();

            pointLatLngs = new List<PointLatLng>();
            pointLatLngs.Add(new PointLatLng(){
                Lat = 33.121450558366,
                Lng = 115.603637695313
            });
            pointLatLngs.Add(new PointLatLng()
            {
                Lat = 34.5065566216456,
                Lng = 113.203125
            });
            pointLatLngs.Add(new PointLatLng()
            {
                Lat = 33.04550781491,
                Lng = 112.47802734375
            });
          
            pointLatLngs.Add(new PointLatLng()
            {
                Lat = 33.121450558366,
                Lng = 115.603637695313
            });        
        }
        #endregion

        /// <summary>
        /// 清除航点信息
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnClearWayPoint_Click(object sender, RoutedEventArgs e)
        {
            IsFirstClickForPloylay = true;
            pointLatLngs.Clear();
            this.MyMapControl.Markers.Clear();
            LoadMap();
        }
        /// <summary>
        /// 绘制DGV
        /// </summary>
        private void WriteDVG()
        {
            dgInfo.ItemsSource = null;
            dgInfo.ItemsSource = wayPointInfoList;

        }
        /// <summary>
        /// 初始化端口
        /// </summary>
        /// <param name="name">端口名称</param>
        /// <param name="baudRate">波特率</param>
        /// <returns></returns>
        public bool InisPort(string name, int baudRate)
        {
            try
            {
                Global.serialPort = sp;
                sp.DataBits = 8;
                sp.DiscardNull = false;
                sp.DtrEnable = false;
                sp.ReceivedBytesThreshold = 1;
                sp.RtsEnable = false;
                sp.StopBits = StopBits.One;
                if (sp.IsOpen)
                {
                    sp.Close();
                }
                sp.WriteBufferSize = 2048;
                sp.BaudRate = baudRate;
                sp.PortName = name;
                sp.DataReceived += new SerialDataReceivedEventHandler(serialPort_DataReceived);
                sp.Open();
            }
            catch (Exception ex)
            {
                Global.isConn = false;
                Global.serialPort = null;

                MessageBox.Show("错误信息：" + ex.Message);
                return false;
            }
            return true;
        }


        #region 接受和解析数据
        private void serialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            bufferNum = sp.BytesToRead;
            byte[] buffer = new byte[bufferNum];
            //读取数据缓冲区的所有数据
            sp.Read(buffer, 0, bufferNum);
            mavDataList.AddRange(buffer);
            while (mavDataList.Count > 2)//
            {
                //第一个是254 且 长度够一个数据帧 (不一定是一个正确帧)
                if (mavDataList[0] == 254 && mavDataList.Count >= 8 + mavDataList[1])
                {
                    byte dataLength = mavDataList[1];
                    mavLinkFrame = new byte[8 + dataLength];
                    mavLinkFrame = mavDataList.GetRange(0, 8 + dataLength).ToArray();
                    dataFrame_2 = new MavLink.MavLinkFrame_2(mavLinkFrame);
                    ushort checksum = MavlinkCRC.crc_calculate(Helper.StructToByteArray_2(dataFrame_2), 6 + dataLength);
                    checksum = MavlinkCRC.crc_accumulate(MavlinkCRC.MAVLINK_MESSAGE_CRCS[mavLinkFrame[5]], checksum);
                    byte ck_a = (byte)(checksum & 0xFF);
                    byte ck_b = (byte)(checksum >> 8);
                    //254是数据部分的254  不是头
                    if (mavLinkFrame[mavLinkFrame.Length - 2] != ck_a || mavLinkFrame[mavLinkFrame.Length - 1] != ck_b)
                    {
                        mavDataList.RemoveAt(0);
                        continue;
                    }
                    else//正确的帧
                    {
                        analyzeMavLinkFrame = new AnalyzeMavLinkFrame(UpdateMavStatus);
                        analyzeMavLinkFrame(mavLinkFrame);
                        //移除这个帧
                        mavDataList.RemoveRange(0, 8 + dataLength);
                    }
                }
                //第一个是254 但 长度不够一个数据帧
                else if (mavDataList[0] == 254 && mavDataList.Count < 8 + mavDataList[1])
                {
                    //结束  继续接收数据
                    break;
                }
                //第一个不是 254
                else
                {
                    //将第一个移除
                    mavDataList.RemoveAt(0);
                    continue;
                }
            }
        }
        #endregion

        /// <summary>
        /// 更新状态
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        public void UpdateMavStatus(byte[] mavLinkFrame)
        {
            switch (mavLinkFrame[5])
            {
                case 0://心跳包
                    UpdateHeartPack(mavLinkFrame);
                    break;
                case 1://系统状态
                    UpdateSysStatus(mavLinkFrame);
                    break;
                case 2://系统时间
                    UpdateSystemTime(mavLinkFrame);
                    break;
                case 22://参数数值
                    UpdateParamValue(mavLinkFrame);
                    break;
                case 24://GPS信息
                    UpdateGPSRow(mavLinkFrame);
                    break;
                case 27://原始的姿态传感器信息
                    UpdateRawImuT(mavLinkFrame);
                    break;
                case 30://姿态信息
                    UpdateAttitude(mavLinkFrame);
                    break;
                case 33://位置信息
                    UpdateGlobalPosition(mavLinkFrame);
                    break;
                case 39://任务项
                    Mission_Iteam(mavLinkFrame);
                    break;
                case 40://任务下载请求
                    Mission_Request(mavLinkFrame);
                    break;
                case 44://任务计数
                    Mission_Count(mavLinkFrame);
                    break;
                case 47://任务回应
                    Mission_Ack(mavLinkFrame);
                    break;
                case 62://导航信息输出
                    UpdateControllerOutput(mavLinkFrame);
                    break;
                case 74://通常用在HUD显示上的各种数据
                    UpdateHUDInfo(mavLinkFrame);
                    break;
                case 77://命令应答  
                    Command_ACK(mavLinkFrame);
                    break;
                case 116:
                    UpdateRawImu2T(mavLinkFrame);
                    break;
                case 150:
                    UpdateSensoroffset(mavLinkFrame);
                    break;
                case 253://文本信息
                    UpdateStatusText(mavLinkFrame);
                    break;
            }
        }
        /// <summary>
        /// 更新心跳包
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateHeartPack(byte[] mavLinkFrame)
        {
            if (Helper.IsEqual(mavLinkFrame, oldHeartPack))
            {
                return;
            }
            else
            {
                oldHeartPack = mavLinkFrame;
                mavStatus.heart_Pack.custom_mode = (uint)(Helper.Turn(mavLinkFrame, 6, 9, DataType.U_Int));
                mavStatus.heart_Pack.type = mavLinkFrame[10];
                mavStatus.heart_Pack.autopilot = mavLinkFrame[11];
                mavStatus.heart_Pack.base_mode = mavLinkFrame[12];
                mavStatus.heart_Pack.system_status = mavLinkFrame[13];
                mavStatus.heart_Pack.mavlink_version = mavLinkFrame[14];
                Global.sysID = mavLinkFrame[3];
                Global.compID = mavLinkFrame[4];//在这里获得的
                if ((mavStatus.heart_Pack.base_mode & 0x80) == 0)
                {
                    isSuo = true;
                    try
                    {//暂时
                        this.Dispatcher.Invoke(new Action(() => this.flyinfo.labSuo.Content = "上锁"));
                    }
                    catch { }

                }
                else
                {
                    isSuo = false;
                    try
                    {
                        //暂时
                        this.Dispatcher.Invoke(new Action(() => this.flyinfo.labSuo.Content = "解锁"));
                    }
                    catch { }

                }
                try
                {
                    switch (mavStatus.heart_Pack.custom_mode)
                    {
                        //暂时
                        case 0:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "自稳"));
                            //          this.Dispatcher.Invoke(new Action(() => this.flyinfo..mode = "自稳"));
                            break;
                        case 2:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "定高"));
                            //        this.Dispatcher.Invoke(new Action(() => this.hud.mode = "定高"));
                            break;
                        case 3:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "任务/自动"));
                            //       this.Dispatcher.Invoke(new Action(() => this.hud.mode = "任务/自动"));
                            break;
                        case 4:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "引导"));
                            //  this.Dispatcher.Invoke(new Action(() => this.hud.mode = "引导"));
                            break;
                        case 5:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "停留"));
                            //   this.Dispatcher.Invoke(new Action(() => this.hud.mode = "停留"));
                            break;
                        case 6:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "返航"));
                            //   this.Dispatcher.Invoke(new Action(() => this.hud.mode = "返航"));
                            break;
                        case 7:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "环绕"));
                            //   this.Dispatcher.Invoke(new Action(() => this.hud.mode = "环绕"));
                            break;
                        case 9:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "降落"));
                            //  this.Dispatcher.Invoke(new Action(() => this.hud.mode = "降落"));
                            break;
                        case 11:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "漂移"));
                            //  this.Dispatcher.Invoke(new Action(() => this.hud.mode = "漂移"));
                            break;
                        case 13:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "运动"));
                            //  this.Dispatcher.Invoke(new Action(() => this.hud.mode = "运动"));
                            break;
                        case 16:
                            this.Dispatcher.Invoke(new Action(() => this.flyinfo.labState.Content = "定点"));
                            //  this.Dispatcher.Invoke(new Action(() => this.hud.mode = "定点"));
                            break;
                    }
                }
                catch { }
            }
        }

        /// <summary>
        /// 更新系统状态
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateSysStatus(byte[] mavLinkFrame)
        {

            mavStatus.sys_Status.onboard_control_sensors_present = (UInt32)(Helper.Turn(mavLinkFrame, 6, 9, DataType.U_Int));
            mavStatus.sys_Status.onboard_control_sensors_enabled = (UInt32)(Helper.Turn(mavLinkFrame, 10, 13, DataType.U_Int));
            mavStatus.sys_Status.onboard_control_sensors_health = (UInt32)(Helper.Turn(mavLinkFrame, 14, 17, DataType.U_Int));
            mavStatus.sys_Status.load = (ushort)(Helper.Turn(mavLinkFrame, 18, 19, DataType.U_Short));
            mavStatus.sys_Status.voltage_battery = (ushort)(Helper.Turn(mavLinkFrame, 20, 21, DataType.U_Short));
            //mavStatus.sys_Status.voltage_battery = (ushort)(Helper.Turn(mavLinkFrame, 20, 21, DataType.U_Short));
            mavStatus.sys_Status.current_battery = (short)(Helper.Turn(mavLinkFrame, 22, 23, DataType.S_Short));


            mavStatus.sys_Status.drop_rate_comm = (ushort)(Helper.Turn(mavLinkFrame, 24, 25, DataType.U_Short));
            mavStatus.sys_Status.errors_comm = (ushort)(Helper.Turn(mavLinkFrame, 26, 27, DataType.U_Short));
            mavStatus.sys_Status.errors_count1 = (ushort)(Helper.Turn(mavLinkFrame, 28, 29, DataType.U_Short));
            mavStatus.sys_Status.errors_count2 = (ushort)(Helper.Turn(mavLinkFrame, 30, 31, DataType.U_Short));
            mavStatus.sys_Status.errors_count3 = (ushort)(Helper.Turn(mavLinkFrame, 32, 33, DataType.U_Short));
            mavStatus.sys_Status.errors_count4 = (ushort)(Helper.Turn(mavLinkFrame, 34, 35, DataType.U_Short));
            mavStatus.sys_Status.battery_remaining = mavLinkFrame[36];
            //暂时
            this.Dispatcher.Invoke(new Action(() =>
            {
                //丢包率
                this.flyinfo.labLostLv.Content = mavStatus.sys_Status.drop_rate_comm.ToString();
                this.flyinfo.txtThrowPackage.Text = mavStatus.sys_Status.drop_rate_comm.ToString();
                //电池电压
                this.flyinfo.labDianYa.Content = mavStatus.sys_Status.voltage_battery.ToString();
                //电池余量
                this.flyinfo.labYuLiang.Content = mavStatus.sys_Status.battery_remaining.ToString();
                //丢包计数
                this.flyinfo.labLostNum.Content = mavStatus.sys_Status.errors_comm.ToString();
            }));
        }
      
        /// <summary>
        /// 更新系统时间
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateSystemTime(byte[] mavLinkFrame)
        {
            mavStatus.system_Time.time_boot_ms = (uint)Helper.Turn(mavLinkFrame, 14, 17, DataType.U_Int);
            mavStatus.system_Time.time_unix_usec = (UInt64)Helper.Turn(mavLinkFrame, 6, 13, DataType.U_Int64);
        }
     
        /// <summary>
        /// 更新参数数值
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateParamValue(byte[] mavLinkFrame)
        {
            mavStatus.param_Value.param_value = (float)Helper.Turn(mavLinkFrame, 6, 9, DataType.Float);
            mavStatus.param_Value.param_count = (ushort)Helper.Turn(mavLinkFrame, 10, 11, DataType.U_Short);
            mavStatus.param_Value.param_index = (ushort)Helper.Turn(mavLinkFrame, 12, 13, DataType.U_Short);
            mavStatus.param_Value.param_id = new byte[16];
            for (int i = 0; i < 16; i++)
            {
                mavStatus.param_Value.param_id[i] = mavLinkFrame[i + 14];
            }
            mavStatus.param_Value.param_type = mavLinkFrame[30];



            string paramID = System.Text.ASCIIEncoding.ASCII.GetString(mavStatus.param_Value.param_id);
            int pos = paramID.IndexOf('\0');
            if (pos != -1)
            {
                paramID = paramID.Substring(0, pos);
            }
            if (!mavStatus.paraDic.ContainsKey(paramID))//如果不包含 就将数据添加进去
            {
                mavStatus.paraDic.Add(paramID, mavStatus.param_Value.param_value);
            }
            //如果包含 但是值被更新了，就更新paramFrameDic中的值，保持最新。
            if (mavStatus.paraDic.ContainsKey(paramID) && mavStatus.paraDic[paramID] != mavStatus.param_Value.param_value)
            {
                mavStatus.paraDic[paramID] = mavStatus.param_Value.param_value;
            }
        }
        
        /// <summary>
        /// 更新GPS信息
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateGPSRow(byte[] mavLinkFrame)
        {
            //pointsList.Add(new PointLatLng(mavStatus.gps_Row.lat / Math.Pow(10, 7), mavStatus.gps_Row.lng / Math.Pow(10, 7)));
            mavStatus.gps_Row.time_usec = (UInt64)Helper.Turn(mavLinkFrame, 6, 13, DataType.U_Int64);
            //mavStatus.gps_Row.fix_type = mavLinkFrame[14];
            mavStatus.gps_Row.lat = (Int32)Helper.Turn(mavLinkFrame, 14, 17, DataType.S_Int);
            mavStatus.gps_Row.lng = (int)Helper.Turn(mavLinkFrame, 18, 21, DataType.S_Int);
            mavStatus.gps_Row.alt = (int)Helper.Turn(mavLinkFrame, 22, 25, DataType.S_Int);
            mavStatus.gps_Row.eph = (ushort)Helper.Turn(mavLinkFrame, 26, 27, DataType.U_Short);
            mavStatus.gps_Row.epv = (ushort)Helper.Turn(mavLinkFrame, 28, 29, DataType.U_Short);
            mavStatus.gps_Row.vel = (ushort)Helper.Turn(mavLinkFrame, 30, 31, DataType.U_Short);
            mavStatus.gps_Row.cog = (ushort)Helper.Turn(mavLinkFrame, 32, 33, DataType.U_Short);

            mavStatus.gps_Row.fix_type = mavLinkFrame[34];
            mavStatus.gps_Row.satellites_visible = mavLinkFrame[35];
            try
            {
                //暂时
                this.Dispatcher.Invoke(new Action(() =>
                {
                    this.flyinfo.labNum.Content = mavStatus.gps_Row.satellites_visible.ToString();
                    this.flyinfo.labJingDu.Content = mavStatus.gps_Row.eph.ToString();

                }));
            }
            catch { };
            if (isOpen)
            {
                //暂时
                /* Global.nowLocation = new PointLatLng(mavStatus.gps_Row.lat / Math.Pow(10, 7), mavStatus.gps_Row.lng / Math.Pow(10, 7));
                 this.Invoke(new Action(() =>
                 {
                     MainMap.Position = Global.nowLocation;
                     MainMap.Zoom = 14;
                     pointsList.Add(new PointLatLng(mavStatus.gps_Row.lat / Math.Pow(10, 7), mavStatus.gps_Row.lng / Math.Pow(10, 7)));
                     DrawRouite(new PointLatLngAlt(mavStatus.gps_Row.lat / Math.Pow(10, 7), mavStatus.gps_Row.lng / Math.Pow(10, 7), mavStatus.gps_Row.alt / Math.Pow(10, 7)));//绘制行走轨迹

                 }));

                 isOpen = false;*/
            }
        }

        /// <summary>
        /// 原始的姿态传感器信息
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateRawImuT(byte[] mavLinkFrame)
        {
            mavStatus.raw_imu_t.time_usec = (UInt64)Helper.Turn(mavLinkFrame, 6, 13, DataType.U_Int64);
            mavStatus.raw_imu_t.xacc = (Int16)Helper.Turn(mavLinkFrame, 14, 15, DataType.S_Short);
            mavStatus.raw_imu_t.yacc = (Int16)Helper.Turn(mavLinkFrame, 16, 17, DataType.S_Short);
            mavStatus.raw_imu_t.zacc = (Int16)Helper.Turn(mavLinkFrame, 18, 19, DataType.S_Short);
            mavStatus.raw_imu_t.xgyro = (Int16)Helper.Turn(mavLinkFrame, 20, 21, DataType.S_Short);
            mavStatus.raw_imu_t.ygyro = (Int16)Helper.Turn(mavLinkFrame, 22, 23, DataType.S_Short);
            mavStatus.raw_imu_t.zgyro = (Int16)Helper.Turn(mavLinkFrame, 24, 25, DataType.S_Short);
            mavStatus.raw_imu_t.xmag = (Int16)Helper.Turn(mavLinkFrame, 26, 27, DataType.S_Short);
            mavStatus.raw_imu_t.ymag = (Int16)Helper.Turn(mavLinkFrame, 28, 29, DataType.S_Short);
            mavStatus.raw_imu_t.zmag = (Int16)Helper.Turn(mavLinkFrame, 30, 31, DataType.S_Short);

            float rawmx = mavStatus.raw_imu_t.xmag - (float)mavStatus.sensor_offset.mag_ofs_x;
            float rawmy = mavStatus.raw_imu_t.ymag - (float)mavStatus.sensor_offset.mag_ofs_y;
            float rawmz = mavStatus.raw_imu_t.zmag - (float)mavStatus.sensor_offset.mag_ofs_z;
            sba.Append("list.Add(point = new Point() { X =" + rawmx);
            sba.Append(", Y = " + rawmy + "});");


            //暂时

            /*   if (SanHeGroundStation.Forms.ProgressReporterSphereUsing.MagCalib.boostart)
               {
                   SanHeGroundStation.Forms.ProgressReporterSphereUsing.MagCalib.comdatacompass1 = new Tuple<float, float, float>(rawmx, rawmy, rawmz);

               } */


        }

        /// <summary>
        /// 姿态信息
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateAttitude(byte[] mavLinkFrame)
        {
            mavStatus.attitude.time_boot_ms = (uint)Helper.Turn(mavLinkFrame, 6, 9, DataType.U_Int);
            mavStatus.attitude.roll = Helper.RadianToAngle((float)Helper.Turn(mavLinkFrame, 10, 13, DataType.Float));
            mavStatus.attitude.pitch = Helper.RadianToAngle((float)Helper.Turn(mavLinkFrame, 14, 17, DataType.Float));
            mavStatus.attitude.yaw = Helper.RadianToAngle((float)Helper.Turn(mavLinkFrame, 18, 21, DataType.Float));
            mavStatus.attitude.rollspeed = Helper.RadianToAngle((float)Helper.Turn(mavLinkFrame, 22, 25, DataType.Float));
            mavStatus.attitude.pitchspeed = Helper.RadianToAngle((float)Helper.Turn(mavLinkFrame, 26, 29, DataType.Float));
            mavStatus.attitude.yawspeed = Helper.RadianToAngle((Single)Helper.Turn(mavLinkFrame, 30, 33, DataType.Float));
            //暂时
            try
            {
                this.Dispatcher.Invoke(new Action(() =>
                {
                    this.hud.RollAngle = mavStatus.attitude.roll;
                    this.hud.PitchAngle = mavStatus.attitude.pitch;
                    this.hud.YanAngle = mavStatus.attitude.yaw;
                    this.flyinfo.labHengGunJiao.Content = mavStatus.attitude.roll.ToString() + "°";
                    this.flyinfo.labFuYangJiao.Content = mavStatus.attitude.pitch.ToString() + "°";
                    this.flyinfo.labFuYangJiao.Content = mavStatus.attitude.pitch.ToString() + "°";
                    this.flyinfo.txtFYJ.Text = mavStatus.attitude.pitch.ToString() + "°";
                    this.flyinfo.labHengGunJiaoSpeed.Content = mavStatus.attitude.rollspeed.ToString() + "°";
                    this.flyinfo.labFuYangJiaoSpeed.Content = mavStatus.attitude.pitchspeed.ToString() + "°";
                    this.flyinfo.txtGspeed.Text = mavStatus.attitude.pitchspeed.ToString() + "°";

                }));

            }
            catch
            {
                return;
            }
        }

        /// <summary>
        /// 位置信息
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateGlobalPosition(byte[] mavLinkFrame)
        {
            mavStatus.global_Position.time_boot_ms = (UInt32)Helper.Turn(mavLinkFrame, 6, 9, DataType.U_Int);
            mavStatus.global_Position.lat = (Int32)Helper.Turn(mavLinkFrame, 10, 13, DataType.S_Int);
            mavStatus.global_Position.lon = (int)Helper.Turn(mavLinkFrame, 14, 17, DataType.S_Int);
            mavStatus.global_Position.alt = (int)Helper.Turn(mavLinkFrame, 18, 21, DataType.S_Int);
            mavStatus.global_Position.relative_alt = (int)Helper.Turn(mavLinkFrame, 22, 25, DataType.S_Int);
            mavStatus.global_Position.vx = (short)Helper.Turn(mavLinkFrame, 26, 27, DataType.S_Short);
            mavStatus.global_Position.vy = (short)Helper.Turn(mavLinkFrame, 28, 29, DataType.S_Short);
            mavStatus.global_Position.vz = (short)Helper.Turn(mavLinkFrame, 30, 31, DataType.S_Short);
            mavStatus.global_Position.hdg = (ushort)Helper.Turn(mavLinkFrame, 32, 33, DataType.U_Short);
        }

        /// <summary>
        /// 任务项
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void Mission_Iteam(byte[] mavLinkFrame)
        {
            mission_iteam.param1 = (float)Helper.Turn(mavLinkFrame, 6, 9, DataType.Float);
            mission_iteam.param2 = (float)Helper.Turn(mavLinkFrame, 10, 13, DataType.Float);
            mission_iteam.param3 = (float)Helper.Turn(mavLinkFrame, 14, 17, DataType.Float);
            mission_iteam.param4 = (float)Helper.Turn(mavLinkFrame, 18, 21, DataType.Float);
            mission_iteam.x = (float)Helper.Turn(mavLinkFrame, 22, 25, DataType.Float);
            mission_iteam.y = (float)Helper.Turn(mavLinkFrame, 26, 29, DataType.Float);
            mission_iteam.z = (float)Helper.Turn(mavLinkFrame, 30, 33, DataType.Float);
            mission_iteam.seq = (ushort)Helper.Turn(mavLinkFrame, 34, 35, DataType.U_Short);
            mission_iteam.command = (ushort)Helper.Turn(mavLinkFrame, 36, 37, DataType.U_Short);
            mission_iteam.target_system = mavLinkFrame[38];
            mission_iteam.target_component = mavLinkFrame[39];
            mission_iteam.frame = mavLinkFrame[40];
            mission_iteam.current = mavLinkFrame[41];
            mission_iteam.autocontinue = mavLinkFrame[42];
            if (index == mission_iteam.seq)
            {
                MessageBox.Show(mission_iteam.seq.ToString() + "成功接收到了。");
                LocationWayPoint wps = new LocationWayPoint();
                wps.p1 = mission_iteam.param1;
                wps.p2 = mission_iteam.param2;
                wps.p3 = mission_iteam.param3;
                wps.p4 = mission_iteam.param4;
                wps.id = (byte)mission_iteam.command;
                wps.lat = mission_iteam.x;
                wps.lng = mission_iteam.y;
                wps.alt = mission_iteam.z;
                wps.options = (byte)(mission_iteam.frame);
                cmdlist.Add(wps);
                if (cmdcount > cmdlist.Count)
                {
                    index++;
                    getWP((ushort)(index));
                }
                else
                {
                    index = 0;
                    mission_iteam.seq = 0;
                    wpslisttopointlist(cmdlist, 1);
                    //  暂时  drawDgv(cmdlist, 1);                    
                    DrawPolygon(pointlatlngaltList);
                }
            }
        }

        /// <summary>
        /// 任务下载请求,40号解析收到的序号
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void Mission_Request(byte[] mavLinkFrame)
        {
            mission_request.seq = (ushort)Helper.Turn(mavLinkFrame, 6, 7, DataType.U_Short);
            mission_request.target_system = mavLinkFrame[8];
            mission_request.target_component = mavLinkFrame[9];

            List<LocationWayPoint> missionWayPointList = LatlonAltToLocationWayPoints1();

            MAVLink.MavLink.MAV_FRAME frame = MAVLink.MavLink.MAV_FRAME.GLOBAL_RELATIVE_ALT;
            if (mission_request.seq == 0)
            {
                SendWP(missionWayPointList[mission_request.seq], (ushort)mission_request.seq, MAVLink.MavLink.MAV_FRAME.GLOBAL, 0);
                now = DateTime.Now;
            }
            else
            {
                SendWP(missionWayPointList[mission_request.seq], (ushort)mission_request.seq, frame, 0);

                if (mission_request.seq == missionWayPointList.Count - 1)
                {
                    isOK = true;
                    MessageBox.Show("写入成功！");

                }
                else if (mission_request.seq < missionWayPointList.Count - 1)
                {
                    now = DateTime.Now;
                }
            }

        }

        /// <summary>
        /// 任务计数
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void Mission_Count(byte[] mavLinkFrame)
        {
            mission_count.count = (ushort)Helper.Turn(mavLinkFrame, 6, 7, DataType.U_Short);
            mission_count.target_system = mavLinkFrame[8];
            mission_count.target_component = mavLinkFrame[9];
            // MessageBox.Show("航点总数：" + mission_count.count.ToString());
            cmdcount = mission_count.count;//这两个是全局变量
            index = 0;
            getWP(index);//开始要第0个
            getWP(index);//要两次增加正确的机率。


        }

        /// <summary>
        /// 任务回应
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void Mission_Ack(byte[] mavLinkFrame)
        {
            mission_Ack.target_system = mavLinkFrame[6];
            mission_Ack.target_component = mavLinkFrame[7];
            mission_Ack.type = mavLinkFrame[8];

        }

        /// <summary>
        /// 导航信息输出
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateControllerOutput(byte[] mavLinkFrame)
        {
            mavStatus.nav_Controller_Output.nav_roll = (float)Helper.Turn(mavLinkFrame, 6, 9, DataType.Float);
            mavStatus.nav_Controller_Output.nav_pitch = (float)Helper.Turn(mavLinkFrame, 10, 13, DataType.Float);

            mavStatus.nav_Controller_Output.alt_error = (float)Helper.Turn(mavLinkFrame, 14, 17, DataType.Float);
            mavStatus.nav_Controller_Output.aspd_error = (float)Helper.Turn(mavLinkFrame, 18, 21, DataType.Float);
            mavStatus.nav_Controller_Output.xtrack_error = (float)Helper.Turn(mavLinkFrame, 22, 25, DataType.Float);

            mavStatus.nav_Controller_Output.nav_bearing = (short)Helper.Turn(mavLinkFrame, 26, 27, DataType.S_Short);
            mavStatus.nav_Controller_Output.target_bearing = (short)Helper.Turn(mavLinkFrame, 28, 29, DataType.S_Short);
            mavStatus.nav_Controller_Output.wp_dist = (ushort)Helper.Turn(mavLinkFrame, 30, 31, DataType.U_Short);
        }

        /// <summary>
        /// 通常用在HUD显示上的各种数据
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateHUDInfo(byte[] mavLinkFrame)
        {
            mavStatus.Vfr_Hud.airspeed = (float)Helper.Turn(mavLinkFrame, 6, 9, DataType.Float);
            mavStatus.Vfr_Hud.groundspeed = (float)Helper.Turn(mavLinkFrame, 10, 13, DataType.Float);
            mavStatus.Vfr_Hud.alt = (float)Helper.Turn(mavLinkFrame, 14, 17, DataType.Float);
            mavStatus.Vfr_Hud.climb = (float)Helper.Turn(mavLinkFrame, 18, 21, DataType.Float);
            mavStatus.Vfr_Hud.heading = (short)Helper.Turn(mavLinkFrame, 22, 23, DataType.S_Short);
            mavStatus.Vfr_Hud.throttle = (ushort)Helper.Turn(mavLinkFrame, 24, 25, DataType.U_Short);


            try
            {
                this.Dispatcher.Invoke(new Action(() =>
                {
                    //   this.hud.alt = mavStatus.Vfr_Hud.alt;
                    this.hud.FlyHeight = mavStatus.Vfr_Hud.alt;
                    this.flyinfo.labAlt.Content = mavStatus.Vfr_Hud.alt.ToString() + "m";
                    this.flyinfo.txtAlt.Text = mavStatus.Vfr_Hud.alt.ToString() + "m";
                    this.flyinfo.labSpeed.Content = mavStatus.Vfr_Hud.groundspeed.ToString() + "m/s";
                    this.flyinfo.txtSpeed.Text = mavStatus.Vfr_Hud.groundspeed.ToString() + "m/s";
                    this.flyinfo.labShangShenSpeed.Content = mavStatus.Vfr_Hud.climb.ToString("f2") + "m/s";
                    /*         this.txtUpSpeed.Text = mavStatus.Vfr_Hud.climb.ToString("f2") + "m/s";
                             this.hud.speed = mavStatus.Vfr_Hud.groundspeed;
                             this.hud.Refresh();
                             this.hud.Invalidate();*/
                    //暂时
                }
                ));
            }
            catch
            {
            }
        }

        /// <summary>
        /// 命令应答  
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void Command_ACK(byte[] mavLinkFrame)
        {
            command_Ack.command = (ushort)Helper.Turn(mavLinkFrame, 6, 7, DataType.U_Short);
            command_Ack.result = mavLinkFrame[8];
        }

        /// <summary>
        /// 更新磁场，加速度
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateRawImu2T(byte[] mavLinkFrame)
        {
            mavStatus.raw_imu2_t.time_usec = (UInt32)Helper.Turn(mavLinkFrame, 6, 9, DataType.U_Int);
            mavStatus.raw_imu2_t.xacc = (Int16)Helper.Turn(mavLinkFrame, 10, 11, DataType.S_Short);
            mavStatus.raw_imu2_t.yacc = (Int16)Helper.Turn(mavLinkFrame, 12, 13, DataType.S_Short);
            mavStatus.raw_imu2_t.zacc = (Int16)Helper.Turn(mavLinkFrame, 14, 15, DataType.S_Short);
            mavStatus.raw_imu2_t.xgyro = (Int16)Helper.Turn(mavLinkFrame, 16, 17, DataType.S_Short);
            mavStatus.raw_imu2_t.ygyro = (Int16)Helper.Turn(mavLinkFrame, 18, 19, DataType.S_Short);
            mavStatus.raw_imu2_t.zgyro = (Int16)Helper.Turn(mavLinkFrame, 20, 21, DataType.S_Short);
            mavStatus.raw_imu2_t.xmag = (Int16)Helper.Turn(mavLinkFrame, 22, 23, DataType.S_Short);
            mavStatus.raw_imu2_t.ymag = (Int16)Helper.Turn(mavLinkFrame, 24, 25, DataType.S_Short);
            mavStatus.raw_imu2_t.zmag = (Int16)Helper.Turn(mavLinkFrame, 26, 27, DataType.S_Short);

            float rawmx = mavStatus.raw_imu2_t.xmag - (float)mavStatus.sensor_offset.mag_ofs_x;
            float rawmy = mavStatus.raw_imu2_t.ymag - (float)mavStatus.sensor_offset.mag_ofs_y;
            float rawmz = mavStatus.raw_imu2_t.zmag - (float)mavStatus.sensor_offset.mag_ofs_z;
            //暂时
            //if (SanHeGroundStation.Forms.ProgressReporterSphereUsing.MagCalib.boostart)
            //{
            //    SanHeGroundStation.Forms.ProgressReporterSphereUsing.MagCalib.comdatacompass2 = new Tuple<float, float, float>(rawmx, rawmy, rawmz);

            //}
        }

        /// <summary>
        /// 传感器的偏移量
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateSensoroffset(byte[] mavLinkFrame)
        {
            mavStatus.sensor_offset.mag_declination = (float)Helper.Turn(mavLinkFrame, 6, 9, DataType.Float);
            mavStatus.sensor_offset.raw_press = (int)Helper.Turn(mavLinkFrame, 10, 13, DataType.S_Int);
            mavStatus.sensor_offset.raw_temp = (int)Helper.Turn(mavLinkFrame, 14, 17, DataType.S_Int);
            mavStatus.sensor_offset.gyro_cal_x = (float)Helper.Turn(mavLinkFrame, 18, 21, DataType.Float);
            mavStatus.sensor_offset.gyro_cal_y = (float)Helper.Turn(mavLinkFrame, 22, 25, DataType.Float);
            mavStatus.sensor_offset.gyro_cal_z = (float)Helper.Turn(mavLinkFrame, 26, 29, DataType.Float);
            mavStatus.sensor_offset.accel_cal_x = (float)Helper.Turn(mavLinkFrame, 30, 33, DataType.Float);
            mavStatus.sensor_offset.accel_cal_y = (float)Helper.Turn(mavLinkFrame, 34, 37, DataType.Float);
            mavStatus.sensor_offset.accel_cal_z = (float)Helper.Turn(mavLinkFrame, 38, 41, DataType.Float);
            mavStatus.sensor_offset.mag_ofs_x = (short)Helper.Turn(mavLinkFrame, 42, 43, DataType.S_Short);
            mavStatus.sensor_offset.mag_ofs_y = (short)Helper.Turn(mavLinkFrame, 44, 45, DataType.S_Short);
            mavStatus.sensor_offset.mag_ofs_z = (short)Helper.Turn(mavLinkFrame, 46, 47, DataType.S_Short);

        }

        /// <summary>
        /// 通过这个得到飞控的详细信息
        /// </summary>
        /// <param name="mavLinkFrame"></param>
        private void UpdateStatusText(byte[] mavLinkFrame)
        {
            mavStatus.status_Text.severity = mavLinkFrame[6];
            mavStatus.status_Text.text = mavLinkFrame.Skip(7).ToArray();
            //暂时
            /*   this.Invoke(new Action(() =>
               {
                   this.NewtxtContent.AppendText(mavStatus.status_Text.severity.ToString() + " " + Encoding.ASCII.GetString(mavStatus.status_Text.text));
                   this.NewtxtContent.AppendText("\r\n");

                   if (configAccelerometerCalibration != null)
                   {
                       configAccelerometerCalibration.setCalibrationAccelInfo(Encoding.ASCII.GetString(mavStatus.status_Text.text));
                   }
               }
                   ));*/
        }

        bool isOK = false;
        DateTime now; 
        ushort index;
        int cmdcount;

    
        /// <summary>
        ///把获得到的航点信息转换成经纬高集合 
        /// </summary>
        /// <param name="cmd"></param>
        /// <param name="check"></param>
        public void wpslisttopointlist(List<LocationWayPoint> cmd, int check)
        {
            pointlatlngaltList.Clear();
            for (int a = check; a < cmd.Count; a++)
            {
                if (cmd[a].id == 22 || cmd[a].id == 20)
                {
                    continue;
                }

                latlngalt = new PointLatLngAlt(cmd[a].lat, cmd[a].lng, cmd[a].alt);
                pointlatlngaltList.Add(latlngalt);

            }
        }

        public void getWP(ushort index)
        {
            MAVLink.MavLink.mavlink_mission_request req = new MavLink.mavlink_mission_request();
            req.target_system = Global.sysID;
            req.target_component = Global.compID;
            req.seq = index;
            AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.MISSION_REQUEST, req);

        }
        public void AssembleAndSendFrame(byte msgID, object structData)
        {
            if (!Global.isConn)
            {
                return;
            }
            byte[] dataPacket = MavlinkUtil.StructureToByteArray(structData);
            byte[] mavLinkFrame = new byte[dataPacket.Length + 8];
            mavLinkFrame[0] = 254;
            mavLinkFrame[1] = (byte)dataPacket.Length;
            mavLinkFrame[2] = Global.seq;
            mavLinkFrame[3] = Global.sysID;
            mavLinkFrame[4] = Global.compID;
            mavLinkFrame[5] = msgID;
            dataPacket.CopyTo(mavLinkFrame, 6);
            //算校验和
            ushort checksum = MavlinkCRC.crc_calculate(mavLinkFrame, mavLinkFrame[1] + 6);
            checksum = MavlinkCRC.crc_accumulate(MavlinkCRC.MAVLINK_MESSAGE_CRCS[msgID], checksum);
            byte ck_a = (byte)(checksum & 0xFF); ///< High byte
            byte ck_b = (byte)(checksum >> 8); ///< Low byte
            mavLinkFrame[mavLinkFrame.Length - 1] = ck_b;
            mavLinkFrame[mavLinkFrame.Length - 2] = ck_a;
            sp.Write(mavLinkFrame, 0, mavLinkFrame.Length);
            if (Global.seq > 255)
            {
                Global.seq = 0;
            }
            else
            {
                Global.seq++;
            }
        }
        private void Window_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            MyMapControl.Width = mapCanvas.ActualWidth;
            MyMapControl.Height = mapCanvas.ActualHeight;
        }


        /// <summary>
        /// 起飞
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MItakeOff_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                return;
            }
            if (!Helper.IsDouble(flyinfo.txtnumFlyAlt.Text.Trim()))
            {
                MessageBox.Show("请输入目标高度");
                return;
            }
            Global.takeOffAlt = Convert.ToInt32(flyinfo.txtnumFlyAlt.Text.Trim());
            DoCommand(MavLink.MAV_CMD.TAKEOFF, 0, 0, 0, 0, 0, 0, Global.takeOffAlt);
        }
        /// <summary>
        /// 降落
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIland_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                MessageBox.Show("没有连接飞行器");
                return;
            }
            DoCommand(MavLink.MAV_CMD.LAND, 0, 0, 0, 0, 0, 0, 0);
        }
        /// <summary>
        /// 返航
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIreturn_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                MessageBox.Show("没有连接飞行器");
                return;
            }
            else if (MessageBox.Show("你想要飞回到起点吗？", "提示", MessageBoxButton.YesNo, MessageBoxImage.Question) == MessageBoxResult.Yes)
            {
                DoCommand(MavLink.MAV_CMD.RETURN_TO_LAUNCH, 0, 0, 1, 0, 0, 0, 0);
                MessageBox.Show("命令写入成功");
            }
            else
            {
                MessageBox.Show("你取消了此操作");
            }

        }
        /// <summary>
        /// 磁罗盘校准
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIcompassCalibration_Click(object sender, RoutedEventArgs e)
        {

        }
        ConfigAccelerometerCalibration configAccelerometerCalibration;
        /// <summary>
        /// 加速度校准
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIaccelerationCalibration_Click(object sender, RoutedEventArgs e)
        {
            configAccelerometerCalibration = new ConfigAccelerometerCalibration();
            configAccelerometerCalibration.btnCalibrationLevelEvent = CalibrationLevelEvent;
            configAccelerometerCalibration.btnCalibrationAccelEvent = CalibrationAccelEvent;
            configAccelerometerCalibration.ShowDialog();
        }
        private bool CalibrationLevelEvent()
        {
            DoCommand(MavLink.MAV_CMD.PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0);
            return true;
        }
        private bool CalibrationAccelEvent(int result)
        {
            if (!Global.alCalibrationLevel)
            {
                Global.alCalibrationLevel = true;
                DoCommand(MavLink.MAV_CMD.PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0);
                return true;
            }
            MavLink.mavlink_Command_Ack req = new MavLink.mavlink_Command_Ack();
            req.command = 1;
            req.result = (byte)result;


            AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.COMMAND_ACK, req);
            return true;
        }
        /// <summary>
        /// 遥控器校准
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIcontrollerCalibration_Click(object sender, RoutedEventArgs e)
        {

        }
        /// <summary>
        /// 飞行模式设置
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIFlightModeSet_Click(object sender, RoutedEventArgs e)
        {
            FlyModel flyModel = new FlyModel();
            flyModel.ShowDialog();
        }
        /// <summary>
        /// 飞控详细信息
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIflightControlDetails_Click(object sender, RoutedEventArgs e)
        {
            if(!beforeSendCommand())
                return ;
            MavDetailInfoForm mavDetailInfoForm = new MavDetailInfoForm();
            mavDetailInfoForm.ShowDialog();
        }
        /// <summary>
        /// 导出飞控参数
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIflightControlParameters_Click(object sender, RoutedEventArgs e)
        {

        }
        /// <summary>
        /// 显示当前位置
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIdisplayLocation_Click(object sender, RoutedEventArgs e)
        {

        }
        /// <summary>
        /// 飞行默认高度
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIdefaultAltitude_Click(object sender, RoutedEventArgs e)
        {

        }
        /// <summary>
        /// 电子围栏
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void MIelectronicFence_Click(object sender, RoutedEventArgs e)
        {

        }
        public Action ConnClick;
        private void BtnConn_Click(object sender, RoutedEventArgs e)
        {
            if (btnConn.Content.ToString() == "连接")
            {
                Global.baudRate = Convert.ToInt32(cmbBaud.SelectedValue);
                Global.portsName = cmbPortName.Text.Trim();
                this.ConnClick = new Action(Conn);//现在并没有触发委托的那个事件，只有点击的时候才触发，现在是给委托绑定事件
                ConnClick();
            }
            else
            {
                if (!Global.isConn)
                {
                    return;
                }
                try
                {
                    sp.Close();
                    sp.Dispose();//注意释放  不然就只能关一次 之后不会在关闭了
                    Global.isConn = false;
                    bufferNum = 0;
                    btnConn.Content = "已断开";

                }
                catch
                {

                }

            }
        }
        private void Conn()
        {
            if (InisPort(Global.portsName, Global.baudRate))//判断初始化是否成功
            {

                bufferNum = 0;
                DateTime now = DateTime.Now;
                while (true)
                {
                    if (bufferNum > 0)
                    {
                        btnConn.Content = "已连接";
                        Global.isConn = true;
                        PointLatLng pointLatLng = new PointLatLng(Global.mavStatus.global_Position.lon, Global.mavStatus.global_Position.lat);
                        Global.nowLocation = pointLatLng;
                        MyMapControl.Position = pointLatLng;
                        Thread.Sleep(1500);
                        break;
                    }
                    else
                    {
                        if (now.AddSeconds(3) < DateTime.Now)
                        {
                            MessageBox.Show("连接失败,请重试!", "错误");
                            Global.isConn = false;
                            Thread.Sleep(1500);
                            break;
                        }
                    }
                }
            }
            else
            {

            }
        }

        private void BtnReadWayPoint_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                return;
            }
            if (pointlatlngaltList.Count > 0)
            {
                if (MessageBox.Show("是否清除原来的航点坐标", "警告", MessageBoxButton.YesNo) != MessageBoxResult.Yes)
                    return;
                else
                {
                    cmdlist.Clear();
                    if (GetWPCount() == 255)//确保一定得到或者一定没有得到
                    {
                        return;
                    }
                }
            }
            else
            {

            }
        }
        public ushort GetWPCount()
        {
            MAVLink.MavLink.mavlink_Mission_Request_List req = new MavLink.mavlink_Mission_Request_List();
            req.target_system = Global.sysID;
            req.target_component = Global.compID;
            AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.MISSION_REQUEST_LIST, req);//只发了一次
            DateTime now = DateTime.Now;
            int num2 = 5;//总共发5次
            while (true)
            {
                if (mission_count.count == 255)//返回来的命令帧没有来
                {
                    if (now.AddMilliseconds(500) > DateTime.Now)//没有超时
                    {
                        continue;
                    }
                    else//超时  重新发送命令，重新计时
                    {
                        if (num2 == 0)
                        {
                            return (ushort)mission_count.count;
                        }
                        AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.MISSION_REQUEST_LIST, req);
                        num2--;
                        now = DateTime.Now;//从新计时
                    }
                }
                else//值发生变化说明收到数据
                {
                    ushort t = mission_count.count;
                    return t;
                }
            }
        }
        private void BtnWriteWayPoint_Click(object sender, RoutedEventArgs e)
        {
            if (!beforeSendCommand())
            {
                return;
            }
            if (pointlatlngaltList.Count > 0)//航点集合
            {
                List<LocationWayPoint> missionWayPointList = LatlonAltToLocationWayPoints();

                ushort index = (ushort)(dgInfo.Items.Count);
                index = 4;
                var total = SendWpsTotal(index);//发送总航点数 这里容易出现错误 一定要确保航点的总数发送正确

                if (!total)
                {
                    //MessageBox.Show("航点总数发送失败");
                    return;
                }
            }
            else//没有发送的点的集合
            {
                MessageBox.Show("请绘制要发送的点！");
                return;
            }

        }
        /// <summary>
        /// 发送航点的总数 
        /// </summary>
        /// <param name="wps_total">航点总数</param>
        /// <returns>返回是否发送成功</returns>
        public bool SendWpsTotal(ushort wps_total)
        {
            MavLink.mavlink_Mission_Count req = new MavLink.mavlink_Mission_Count();
            req.target_component = Global.compID;
            req.target_system = Global.sysID;
            req.count = wps_total;
            AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.MISSION_COUNT, req);



            DateTime now = DateTime.Now;
            int num = 5;//重试的次数
            while (true)
            {
                if (mission_request.seq == 255 && now.AddMilliseconds(500) > DateTime.Now)//没有应答 且 没有超时 继续接受帧
                {
                    continue;
                }
                else if (mission_request.seq == 0)//有应答
                {
                    //mission_request.seq = 255;
                    return true;
                }
                else if (now.AddMilliseconds(500) < DateTime.Now)//超时
                {
                    if (num == 0)
                    {
                        return false;
                    }
                    AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.MISSION_COUNT, req);
                    num--;
                    now = DateTime.Now;
                }
            }

        }

        //根据经纬度赚换成LocationWayPoint集合
        private List<LocationWayPoint> LatlonAltToLocationWayPoints()
        {
            LocationWayPoint temp;
            List<LocationWayPoint> missions = new List<LocationWayPoint>();
            //添加默认的home航点，
            missions.Add(new LocationWayPoint
            {
                alt = 0,
                id = 16,
                lat = 0,
                lng = 0,
                options = 0,
                p1 = 0,
                p2 = 0,
                p3 = 0,
                p4 = 0,
            });

            for (int a = 0; a < dgInfo.Items.Count - 1; a++)
            {
                temp = new LocationWayPoint();
                if (((dgInfo.Columns[0].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim().Contains("WAYPOINT"))
                {
                    temp.id = (byte)MavLink.MAV_CMD.WAYPOINT;
                }
                if (((dgInfo.Columns[0].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim().Contains("TAKEOFF"))
                {
                    temp.id = (byte)MavLink.MAV_CMD.TAKEOFF;
                }
                if (((dgInfo.Columns[0].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim().Contains("LAND"))
                {
                    temp.id = (byte)MavLink.MAV_CMD.LAND;
                }
                if (((dgInfo.Columns[0].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim().Contains("RETURN_TO_LAUNCH"))
                {
                    temp.id = (byte)MavLink.MAV_CMD.RETURN_TO_LAUNCH;
                }
                temp.p1 = (float)0;
                temp.p2 = (float)0;
                temp.p3 = (float)0;
                temp.p4 = (float)0;
                temp.alt = (float)(Convert.ToDouble(((dgInfo.Columns[3].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim()));
                temp.lat = Convert.ToDouble(((dgInfo.Columns[2].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim());
                temp.lng = Convert.ToDouble(((dgInfo.Columns[1].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim());
                missions.Add(temp);
            }

            return missions;
        }

        private List<LocationWayPoint> LatlonAltToLocationWayPoints1()
        {
            LocationWayPoint temp;
            List<LocationWayPoint> missions = new List<LocationWayPoint>();
            //添加默认的home航点，
            missions.Add(new LocationWayPoint
            {
                alt = 0,
                id = 16,
                lat = 0,
                lng = 0,
                options = 0,
                p1 = 0,
                p2 = 0,
                p3 = 0,
                p4 = 0,
            });
            temp = new LocationWayPoint();
            temp.id = (byte)MavLink.MAV_CMD.WAYPOINT;
            temp.p1 = (float)0;
            temp.p2 = (float)0;
            temp.p3 = (float)0;
            temp.p4 = (float)0;
            temp.alt = (float)2;
            temp.lat = (float)33.121450558366;
            temp.lng = (float)115.603637695313;
            missions.Add(temp);
            temp = new LocationWayPoint();
            temp.id = (byte)MavLink.MAV_CMD.WAYPOINT;
            temp.p1 = (float)0;
            temp.p2 = (float)0;
            temp.p3 = (float)0;
            temp.p4 = (float)0;
            temp.alt = (float)2;
            temp.lat = (float)34.5065566216456;
            temp.lng = (float)113.203125;
            missions.Add(temp);
            temp = new LocationWayPoint();
            temp.id = (byte)MavLink.MAV_CMD.WAYPOINT;
            temp.p1 = (float)0;
            temp.p2 = (float)0;
            temp.p3 = (float)0;
            temp.p4 = (float)0;
            temp.alt = (float)2;
            temp.lat = (float)33.04550781491;
            temp.lng = (float)112.47802734375;
            missions.Add(temp);
            /*for (int a = 0; a < dgInfo.Items.Count - 1; a++)
            {
                temp = new LocationWayPoint();
                if (((dgInfo.Columns[0].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim().Contains("WAYPOINT"))
                {
                    temp.id = (byte)MavLink.MAV_CMD.WAYPOINT;
                }
                if (((dgInfo.Columns[0].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim().Contains("TAKEOFF"))
                {
                    temp.id = (byte)MavLink.MAV_CMD.TAKEOFF;
                }
                if (((dgInfo.Columns[0].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim().Contains("LAND"))
                {
                    temp.id = (byte)MavLink.MAV_CMD.LAND;
                }
                if (((dgInfo.Columns[0].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim().Contains("RETURN_TO_LAUNCH"))
                {
                    temp.id = (byte)MavLink.MAV_CMD.RETURN_TO_LAUNCH;
                }
                temp.p1 = (float)0;
                temp.p2 = (float)0;
                temp.p3 = (float)0;
                temp.p4 = (float)0;
                temp.alt = (float)(Convert.ToDouble(((dgInfo.Columns[3].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim()));
                temp.lat = Convert.ToDouble(((dgInfo.Columns[2].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim());
                temp.lng = Convert.ToDouble(((dgInfo.Columns[1].GetCellContent(dgInfo.Items[a])) as TextBlock).Text.Trim());
                missions.Add(temp);
            }*/

            return missions;
        }
        /// <summary>
        /// 在发送命令之前 判断系统是否准备好
        /// </summary>
        /// <returns></returns>
        private bool beforeSendCommand()
        {
            if (!Global.isConn)
            {
                MessageBox.Show("飞行器没有连接!");
                return false;
            }
            return true;
        }
    }
}
