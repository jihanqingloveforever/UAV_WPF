﻿#pragma checksum "..\..\HUD.xaml" "{8829d00f-11b8-4213-878b-770e8597ac16}" "E05253B11FAAFB8FB519D98C7D4FA19EABC620D3C88283E79CD2CB9909A83D56"
//------------------------------------------------------------------------------
// <auto-generated>
//     此代码由工具生成。
//     运行时版本:4.0.30319.42000
//
//     对此文件的更改可能会导致不正确的行为，并且如果
//     重新生成代码，这些更改将会丢失。
// </auto-generated>
//------------------------------------------------------------------------------

using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Automation;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Forms.Integration;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Markup;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Media.TextFormatting;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Shell;
using UAV_WPF;


namespace UAV_WPF {
    
    
    /// <summary>
    /// HUD
    /// </summary>
    public partial class HUD : System.Windows.Controls.UserControl, System.Windows.Markup.IComponentConnector {
        
        
        #line 9 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Grid Grid_Virwport;
        
        #line default
        #line hidden
        
        
        #line 25 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Grid Grid_YanStaff;
        
        #line default
        #line hidden
        
        
        #line 33 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal UAV_WPF.BorderTextLabel TextBlock_YawStaff_Left;
        
        #line default
        #line hidden
        
        
        #line 34 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Canvas Canvas_YawStaff_Left;
        
        #line default
        #line hidden
        
        
        #line 35 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal UAV_WPF.BorderTextLabel TextBlock_YawStaff_Middle;
        
        #line default
        #line hidden
        
        
        #line 36 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Canvas Canvas_YawStaff_Right;
        
        #line default
        #line hidden
        
        
        #line 37 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal UAV_WPF.BorderTextLabel TextBlock_YawStaff_Right;
        
        #line default
        #line hidden
        
        
        #line 38 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Canvas Canvas_YawStaff_Value;
        
        #line default
        #line hidden
        
        
        #line 40 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock Text_YawStaff_Value;
        
        #line default
        #line hidden
        
        
        #line 44 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Grid Grid_Center;
        
        #line default
        #line hidden
        
        
        #line 50 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Canvas Canvas_ViewPortLeft;
        
        #line default
        #line hidden
        
        
        #line 52 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Canvas Canvas_RollCursor;
        
        #line default
        #line hidden
        
        
        #line 53 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock Text_RollStaff_Value;
        
        #line default
        #line hidden
        
        
        #line 56 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Canvas Canvas_ViewPortMiddle;
        
        #line default
        #line hidden
        
        
        #line 59 "..\..\HUD.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Canvas Canvas_ViewPortRight;
        
        #line default
        #line hidden
        
        private bool _contentLoaded;
        
        /// <summary>
        /// InitializeComponent
        /// </summary>
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        public void InitializeComponent() {
            if (_contentLoaded) {
                return;
            }
            _contentLoaded = true;
            System.Uri resourceLocater = new System.Uri("/UAV_WPF;component/hud.xaml", System.UriKind.Relative);
            
            #line 1 "..\..\HUD.xaml"
            System.Windows.Application.LoadComponent(this, resourceLocater);
            
            #line default
            #line hidden
        }
        
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal System.Delegate _CreateDelegate(System.Type delegateType, string handler) {
            return System.Delegate.CreateDelegate(delegateType, this, handler);
        }
        
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        [System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Design", "CA1033:InterfaceMethodsShouldBeCallableByChildTypes")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1800:DoNotCastUnnecessarily")]
        void System.Windows.Markup.IComponentConnector.Connect(int connectionId, object target) {
            switch (connectionId)
            {
            case 1:
            this.Grid_Virwport = ((System.Windows.Controls.Grid)(target));
            return;
            case 2:
            this.Grid_YanStaff = ((System.Windows.Controls.Grid)(target));
            return;
            case 3:
            this.TextBlock_YawStaff_Left = ((UAV_WPF.BorderTextLabel)(target));
            return;
            case 4:
            this.Canvas_YawStaff_Left = ((System.Windows.Controls.Canvas)(target));
            return;
            case 5:
            this.TextBlock_YawStaff_Middle = ((UAV_WPF.BorderTextLabel)(target));
            return;
            case 6:
            this.Canvas_YawStaff_Right = ((System.Windows.Controls.Canvas)(target));
            return;
            case 7:
            this.TextBlock_YawStaff_Right = ((UAV_WPF.BorderTextLabel)(target));
            return;
            case 8:
            this.Canvas_YawStaff_Value = ((System.Windows.Controls.Canvas)(target));
            return;
            case 9:
            this.Text_YawStaff_Value = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 10:
            this.Grid_Center = ((System.Windows.Controls.Grid)(target));
            return;
            case 11:
            this.Canvas_ViewPortLeft = ((System.Windows.Controls.Canvas)(target));
            return;
            case 12:
            this.Canvas_RollCursor = ((System.Windows.Controls.Canvas)(target));
            return;
            case 13:
            this.Text_RollStaff_Value = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 14:
            this.Canvas_ViewPortMiddle = ((System.Windows.Controls.Canvas)(target));
            return;
            case 15:
            this.Canvas_ViewPortRight = ((System.Windows.Controls.Canvas)(target));
            return;
            }
            this._contentLoaded = true;
        }
    }
}

