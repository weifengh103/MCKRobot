using CommunityToolkit.Mvvm.Messaging;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using WHPG.Models;

namespace WHPG.ViewModels
{
    public partial class SenderViewModel : ObservableObject
    {
        private readonly MainViewModel mainViewModel;
        private readonly AllPumpsList allPumps;

        public PumpService? PumpService { get; }

        //public SenderViewModel(MainViewModel mainViewModel, AllPumpsList allPumps) 
        //{

        //    this.mainViewModel = mainViewModel;
        //    this.allPumps = allPumps;
        //}

        public SenderViewModel(MainViewModel mainViewModel, PumpService? pumpService)
        {
            this.mainViewModel = mainViewModel;
            PumpService = pumpService;
        }

        [RelayCommand]
        public void SwitchToViewB()
        {
            //this.mainViewModel.CurrentViewModel = new ReceiverViewModel(this.mainViewModel,this.allPumps);
            this.mainViewModel.CurrentViewModel = new ReceiverViewModel(this.mainViewModel, PumpService);
        }

        [RelayCommand]
        public void AddPump() 
        {
            var newPump = new Pump(123,"adsfsadf");

           //allPumps.AddPump(newPump);
            //WeakReferenceMessenger.Default.Send(new PumpListMessage(allPumps));

            PumpService.PumpCollect.Add(newPump);

        }
    }
}
