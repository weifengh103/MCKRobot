using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Reservoom.Models
{
    public class Reservation
    {
        public RoomID RoomID { get;  }
        public string UserName { get; }
        public DateTime StartTime { get; }
        public DateTime EndTime { get; }

        public TimeSpan Length => EndTime - StartTime;
            
      
        public Reservation(RoomID roomID, string userName, DateTime startTime, DateTime endTime)
        {
            RoomID = roomID;
            UserName = userName;
            StartTime = startTime;
            EndTime = endTime;
        }

        internal bool Conflicts(Reservation reservation)
        {
            if(reservation.RoomID.ToString() == RoomID.ToString())
                return true;
            return reservation.StartTime <EndTime && reservation.EndTime > StartTime;
        }
    }
}
