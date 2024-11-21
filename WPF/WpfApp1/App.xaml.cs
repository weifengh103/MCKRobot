using System.Configuration;
using System.Data;
using System.Windows;
using WHPG2.Models;
using WHPG2.ViewModels;
using WHPG2.Views;

namespace WHPG2
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {

        protected override void OnStartup(StartupEventArgs e)
        {
            //Pump p = new Pump(1, "1");
            //p.StartPump();
            //Thread.Sleep(5000);
            //p.StopPump();
            //var rmp = p.RPM;
            //AllPumps allPumps = new AllPumps();

            MainViewModel mainViewModel = new MainViewModel();
            MainWindow mainWindow = new MainWindow();

            mainWindow.DataContext = mainViewModel;
            mainWindow.Show();

            base.OnStartup(e);
        }
        
        
    }

}
