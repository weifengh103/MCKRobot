using Reservoom.Exceptions;
using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Reservoom.Models
{
    public class ReservationBook
    {
        public readonly  List<Reservation> reservations;

        public ReservationBook()
        {
            reservations = new  List<Reservation>();
        }

        public IEnumerable<Reservation> GetAllReservations()
        {
            return reservations;
        }


        public void AddReservation(Reservation newReservation)
        {
            foreach (Reservation reservation in reservations)
            {
                if (reservation.Conflicts(newReservation))
                {
                    throw new ReservationConflictException("Error: Reservation conflict",newReservation, reservation);
                }
            }
          
            reservations.Add(newReservation);
        }

    }
}
