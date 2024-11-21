
using System.Collections.ObjectModel;
using CommunityToolkit.Mvvm.ComponentModel;
namespace WHPG.Models
{
    public partial class AllPumpsList: ObservableObject
    {
        [ObservableProperty]
        private ObservableCollection<Pump> pumpList = new ObservableCollection<Pump>();

        public void AddPump(Pump pump)
        { PumpList.Add(pump); }
    }
}
