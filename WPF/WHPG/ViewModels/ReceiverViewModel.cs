using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using CommunityToolkit.Mvvm.Messaging;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using WHPG.Models;

namespace WHPG.ViewModels
{
    public partial class ReceiverViewModel : ObservableObject
    {
        private readonly MainViewModel mainViewModel;

        [ObservableProperty]
        private ObservableCollection<Pump> allPumps;



        public ReceiverViewModel(MainViewModel mainViewModel, PumpService? pumpService)
        {
            this.mainViewModel = mainViewModel;
            PumpService = pumpService;
            allPumps = pumpService.PumpCollect;
            //AllPumps = new ObservableCollection<Pump>(pumpService.PumpCollect);
        }

        public PumpService? PumpService { get; }

        [RelayCommand]
        public void SwitchToViewA()
        {
            var ss = AllPumps;
            //this.mainViewModel.CurrentViewModel = new SenderViewModel(this.mainViewModel, this.allPumps);
            this.mainViewModel.CurrentViewModel = new SenderViewModel(this.mainViewModel, PumpService);
        }

        [RelayCommand]
        public void AddPump()
        {
            var newPump = new Pump(123, "adsfsadf");
            AllPumps.Add(newPump);
        }



    }
}
