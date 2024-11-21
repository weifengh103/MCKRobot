
using WHPG2.Models;
using CommunityToolkit.Mvvm.Collections;

using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;

namespace WHPG2.ViewModels
{
    public partial class ConfigPumpViewModel : ObservableObject
    {
        private readonly MainViewModel mainViewModel;

        [ObservableProperty]
        private AllPumps allpumps;

        [ObservableProperty]
        private string name;


        [ObservableProperty]
        private int id;



        public ConfigPumpViewModel(AllPumps allpumps, MainViewModel mainViewModel)
        {
            this.allpumps = allpumps;
            this.mainViewModel = mainViewModel;
        }

        [RelayCommand]

        public void AddPump() 
        {
            Allpumps.AddPump(new Pump(123, "sda;lkfjsad"));
        }

    }
}
