using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WHPG2.Models
{
    public class AllPumps
    {
        public ObservableCollection<Pump>pumps = new ObservableCollection<Pump>();

        public void AddPump(Pump pump)
            { pumps.Add(pump); }

        public void RemovePump(Pump pump)
        { pumps.Remove(pump); }

    }
}
