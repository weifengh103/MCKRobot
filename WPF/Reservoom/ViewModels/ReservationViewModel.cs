using Reservoom.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Reservoom.ViewModels
{
    public class ReservationViewModel: ViewModelBase
    {
        private readonly Reservation reservation;

        public string RoomID => reservation.RoomID?.ToString();
        public string UserName => reservation.UserName;
        public string StartDate => reservation.StartTime.ToString("d");
        public string EndDate => reservation.EndTime.Date.ToString("d");

        public ReservationViewModel(Reservation reservation)
        {
            this.reservation = reservation;
        }
    }
}
