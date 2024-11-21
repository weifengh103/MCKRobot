using Reservoom.Models;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace Reservoom.ViewModels
{
    class ReservationListingViewModel: ViewModelBase
    {
        private readonly ObservableCollection<ReservationViewModel> reservations;
        public ICommand MakeReservationCommand { get; }
        public IEnumerable<ReservationViewModel> Reservations => reservations;
        public ReservationListingViewModel()
        {
            reservations = new ObservableCollection<ReservationViewModel>();
            reservations.Add(new ReservationViewModel(new Reservation(new RoomID(1, 2), "SingletonSean", DateTime.Now, DateTime.Now)));
            reservations.Add(new ReservationViewModel(new Reservation(new RoomID(2, 3), "SingletdfdsaonSean", DateTime.Now, DateTime.Now)));
            reservations.Add(new ReservationViewModel(new Reservation(new RoomID(3, 4), "SingletoxvvcxvcxnSean", DateTime.Now, DateTime.Now)));

        }
    }
}
