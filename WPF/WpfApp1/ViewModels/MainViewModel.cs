using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CommunityToolkit.Mvvm.Collections;
using CommunityToolkit.Mvvm.ComponentModel;
using WHPG2.Models;

namespace WHPG2.ViewModels
{
    public partial class MainViewModel : ObservableObject
    {
        [ObservableProperty]
        private ObservableObject currentViewModel;

        [ObservableProperty]
        private AllPumps allPumps = new AllPumps();
        public ConfigPumpViewModel configPumpViewModel { get; }


        public MainViewModel()
        {

            //AllPumps allPumps = new AllPumps();
             configPumpViewModel = new ConfigPumpViewModel(allPumps,this);
            this.currentViewModel = configPumpViewModel;
        }
    }
}
