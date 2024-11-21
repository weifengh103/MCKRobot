using Reservoom.Exceptions;
using Reservoom.Models;
using Reservoom.ViewModels;
using System.Configuration;
using System.Data;
using System.Windows;

namespace Reservoom
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {

        private readonly Hotel hotel;

        public App()
        {
            hotel = new Hotel("weifeng hotel");
        }

        protected override void OnStartup(StartupEventArgs e)
        {

            MainWindow = new MainWindow()
            {
                DataContext = new MainViewModel(hotel)

            };
            MainWindow.Show();
            base.OnStartup(e);
        }
    }

}
