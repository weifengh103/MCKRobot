using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using WHPG.Models;

namespace WHPG
{
    public class PumpService
    {
        //public PumpService(ObservableCollection<Pump> pumpCollect)
        //{
        //    PumpCollect = pumpCollect;
        //}

        //public ObservableCollection<Pump> PumpCollect { get; set; }

        public ObservableCollection<Pump> PumpCollect { get; set; }

        // Constructor to initialize the collection
        public PumpService()
        {
            PumpCollect = new ObservableCollection<Pump>();
        }

        // Method to add a new pump to the collection
        //public void AddPump(Pump newPump)
        //{
        //    PumpCollect.Add(newPump);
        //}
    }
}
