using MissionPlanner;
using MissionPlanner.Plugin;
using MissionPlanner.Utilities;
using System;
using System.Collections.Generic;
using System.IO;
using System.Windows.Forms;
using System.Diagnostics;
using MissionPlanner.Controls.PreFlight;
using MissionPlanner.Controls;
using System.Linq;
using GMap.NET.WindowsForms.Markers;
using MissionPlanner.Maps;
using GMap.NET;
using GMap.NET.WindowsForms;
using System.Globalization;
using System.Drawing;
using Microsoft.Win32;

namespace OverlayControlsPlugin
{


    public class myOverlay : GMapOverlay
    {
        //Left and top aligned
        public int boxWidth = 100;
        public int boxHeight = 40;
        public int boxYpos = 200;

        public System.Drawing.Rectangle boxGDIRect = new System.Drawing.Rectangle();

        public override void OnRender(IGraphics g)

        {
            //base.OnRender(g);
            g.DrawRectangle(new Pen(Color.Pink, 4), Control.Width / 2 - boxWidth, -(Control.Height / 2) + boxYpos, boxWidth, boxHeight);
            boxGDIRect = new System.Drawing.Rectangle(Control.Width - boxWidth, boxYpos, boxWidth, boxHeight);

        }
    }

    public class OverlayControlsPlugin : Plugin
    {


        internal static myOverlay mo;
        internal int mouseX;
        internal int mouseY;

        public override string Name
        {
            get { return "OverlayControlsPlugin"; }
        }

        public override string Version
        {
            get { return "0.1"; }
        }

        public override string Author
        {
            get { return "Add your name here"; }
        }

        //[DebuggerHidden]
        public override bool Init()
		//Init called when the plugin dll is loaded
        {
            loopratehz = 1;  //Loop runs every second (The value is in Hertz, so 2 means every 500ms, 0.1f means every 10 second...) 

            mo = new myOverlay();
            Host.FDGMapControl.Overlays.Add(mo);

            Host.FDGMapControl.OnMapZoomChanged += FDGMapControl_OnMapZoomChanged;
            Host.FDGMapControl.MouseDown += FDGMapControl_MouseDown;
            Host.FDGMapControl.MouseClick += FDGMapControl_MouseClick;

            return true;	 // If it is false then plugin will not load
        }

        private void FDGMapControl_MouseDown(object sender, MouseEventArgs e)
        {

            mouseX = e.X;
            mouseY = e.Y;
        }

        private void FDGMapControl_MouseClick(object sender, MouseEventArgs e)
        {

                Console.WriteLine("Mouse position {0} {1}", mouseX, mouseY);
                Console.WriteLine("Rect Postion {0} {1}", mo.boxGDIRect.Left, mo.boxGDIRect.Top);

                if (mo.boxGDIRect.Contains(new Point(mouseX, mouseY))) Console.WriteLine("Box clicked !!!!");

        }

        private void FDGMapControl_OnMapZoomChanged()
        {
            Host.FDGMapControl.Position = Host.FDGMapControl.Position;
        }

        public override bool Loaded()
		//Loaded called after the plugin dll successfully loaded
        {
            return true;     //If it is false plugin will not start (loop will not called)
        }

        public override bool Loop()
		//Loop is called in regular intervalls (set by loopratehz)
        {
            return true;	//Return value is not used
        }

        public override bool Exit()
		//Exit called when plugin is terminated (usually when Mission Planner is exiting)
        {
            return true;	//Return value is not used
        }
    }
}