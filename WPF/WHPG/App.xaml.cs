using CommunityToolkit.Mvvm.Input;
using CommunityToolkit.Mvvm.Messaging;

using System.Windows;
using System.Windows.Input;
using WHPG.Models;
using WHPG.ViewModels;
using WHPG.Views;

namespace WHPG
{

    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {
     
        protected override void OnStartup(StartupEventArgs e)
        {
            AllPumpsList allPumps = new AllPumpsList();
            MainViewModel mainViewModel = new MainViewModel(allPumps);
            MainWindow mainWindow = new MainWindow();
            mainWindow.DataContext = mainViewModel;
            mainWindow.Show();
            base.OnStartup(e);
        }




    }


}
