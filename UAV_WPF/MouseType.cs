using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UAV_WPF
{
    /// <summary>
    /// 鼠标类型
    /// </summary>
    public enum MouseType
    {
        /// <summary>
        /// 无
        /// </summary>
        None,
        /// <summary>
        /// 拖拽地图
        /// </summary>
        DragMap,
        /// <summary>
        /// 绘制矩形搜索区域
        /// </summary>
        DrawBound,
        /// <summary>
        /// 绘制测量线条
        /// </summary>
        DrawDistance,
        /// <summary>
        /// 绘制标记点
        /// </summary>
        DrawMarker,
        /// <summary>
        /// 绘制矩形
        /// </summary>
        DrawRectange,
        /// <summary>
        /// 绘制多边形
        /// </summary>
        DrawPolygon,
        /// <summary>
        /// 绘制下载区域
        /// </summary>
        DrawDownloadArea
    }
}