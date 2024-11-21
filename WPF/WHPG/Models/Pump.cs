using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Principal;
using System.Text;
using System.Threading.Tasks;

namespace WHPG.Models
{
    public class Pump
    {
        private int id;

        public int ID
        {
            get { return id; }
            set { id = value; }
        }

        private string name;

        public Pump(int iD, string name)
        {
            ID = iD;
            Name = name;
        }

        public string Name
        {
            get { return name; }
            set { name = value; }
        }

        //private string name;
        //private int id; 
        //public Pump(string name, int id)
        //{
        //    this.name = name;
        //    this.id = id;   
        //}


    }
}
