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

namespace PluginSkeleton
{
    public class PluginSkeleton : Plugin
    {
        #region Plugin info
        public override string Name
        {
            get { return "PluginSkeleton"; }
        }

        public override string Version
        {
            get { return "0.1"; }
        }

        public override string Author
        {
            get { return "Add your name here"; }
        }
        #endregion // Plugin info

        TabPage tabPage = new TabPage();
        FlowLayoutPanel flowPanel = new FlowLayoutPanel();
        Label testLabel = new Label();
        MyButton testButton = new MyButton();
        TextBox testTextBox = new TextBox();

        /// <summary>
        /// Init called when the plugin dll is loaded
        /// </summary>
        public override bool Init()
        {
            loopratehz = 1; // Loop runs every second (The value is in Hertz, so 2 means every 500ms, 0.1f means every 10 seconds...)

            return true; // If this is false the plugin will not load
        }

        /// <summary>
        /// Loaded called when the plugin is activated
        /// </summary>
        public override bool Loaded()
        {
            MainV2.instance.BeginInvoke((MethodInvoker)delegate
            {
                // Setup tabpage
                tabPage.Name = "tabTestTab";
                tabPage.Text = "Test Tab";
                int index = 1;
                List<string> list = Host.config.GetList("tabcontrolactions").ToList();
                list.Insert(index, tabPage.Name);
                Host.config.SetList("tabcontrolactions", list);
                Host.MainForm.FlightData.TabListOriginal.Insert(index, tabPage);
                Host.MainForm.FlightData.tabControlactions.TabPages.Insert(index, tabPage);

                // Setup flow panel
                flowPanel.Name = "flowPanelTestPanel";
                flowPanel.FlowDirection = FlowDirection.TopDown;
                flowPanel.AutoSize = true;
                flowPanel.Dock = DockStyle.Fill;
                flowPanel.WrapContents = false;
                flowPanel.Resize += FlowPanel_Resize;
                tabPage.Controls.Add(flowPanel);

                // Setup controls
                testLabel.Name = "labelTestLabel";
                testLabel.Text = "This is a test label";
                testLabel.TextAlign = ContentAlignment.MiddleLeft;
                testLabel.Padding = new Padding(3);
                testLabel.AutoSize = true;
                flowPanel.Controls.Add(testLabel);

                testButton.Name = "buttonTestButton";
                testButton.Text = "This is a test button";
                testButton.TextAlign = ContentAlignment.MiddleCenter;
                testButton.Padding = new Padding(3);
                testButton.AutoSize = true;
                testButton.Click += TestButton_Click;
                flowPanel.Controls.Add(testButton);

                testTextBox.Name = "textBoxTestTextBox";
                testTextBox.ReadOnly = true;
                testTextBox.Multiline = true;
                testTextBox.ScrollBars = ScrollBars.Vertical;
                testTextBox.AutoSize = true;
                flowPanel.Controls.Add(testTextBox);
                testTextBox.Height = (flowPanel.Bottom - testTextBox.Top) - (testTextBox.Margin.Top + testTextBox.Margin.Bottom); // Fill to bottom

                // Apply theme
                ThemeManager.ApplyThemeTo(tabPage);
                flowPanel.Refresh();
            });

            return true; // If this is false the plugin will not start (loop will not be called)
        }

        /// <summary>
        /// Loop is called in regular intervals (set by loopratehz)
        /// </summary>
        public override bool Loop()
        {
            return true; // Return value is not used
        }

        /// <summary>
        /// Exit called when plugin is terminated (usually when Mission Planner is exiting)
        /// </summary>
        public override bool Exit()
        {
            return true; // Return value is not used
        }

        private void FlowPanel_Resize(object sender, EventArgs e)
        {
            MainV2.instance.BeginInvoke((MethodInvoker)delegate
            {
                // Adjust control widths in flowPanel
                foreach (Control item in flowPanel.Controls)
                    item.Width = flowPanel.ClientSize.Width - (item.Margin.Left + item.Margin.Right);

                testTextBox.Height = (flowPanel.ClientRectangle.Bottom - testTextBox.Top) - (testTextBox.Margin.Top + testTextBox.Margin.Bottom); // Fill to bottom
            });
        }

        private void TestButton_Click(object sender, EventArgs e)
        {
            MainV2.instance.BeginInvoke((MethodInvoker)delegate
            {
                if (testTextBox.Lines.Length > 0) testTextBox.AppendText(Environment.NewLine);
                testTextBox.AppendText($"{DateTime.Now} - Test button was pressed!");
            });
        }
    }
}