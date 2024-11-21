using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Reservoom.Models
{
    public class Hotel
    {
        private readonly ReservationBook reservationBook;

        public string Name { get; }

        public Hotel(string name)
        {
            this.Name = name;
            reservationBook = new ReservationBook();
        }

        public IEnumerable<Reservation> GetAllReservations( )
        {
            return reservationBook.GetAllReservations();
        }

        public void MakeReservation(Reservation reservation)
        {
            reservationBook.AddReservation(reservation);
        }
    }
}
