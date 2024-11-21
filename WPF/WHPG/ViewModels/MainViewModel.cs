using CommunityToolkit.Mvvm.ComponentModel;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using WHPG.Models;

namespace WHPG.ViewModels
{
    public partial class MainViewModel : ObservableObject
    {
        [ObservableProperty]
        private ObservableObject currentViewModel;
        //private AllPumpsList allPumps;

        private PumpService PumpService = new PumpService();

        // ViewModels for ViewA and ViewB
        public SenderViewModel ViewModelA { get; }
        public ReceiverViewModel ViewModelB { get; }

      

        public MainViewModel(AllPumpsList allPumps)
        {
             
            // Initialize ViewModels and pass the MainViewModel instance
            ViewModelA = new SenderViewModel(this, PumpService);
            ViewModelB = new ReceiverViewModel(this, PumpService);

            // Set initial ViewModel
            CurrentViewModel = ViewModelA;
        }

        
    }
}
