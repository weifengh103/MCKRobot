using Reservoom.Models;
using Reservoom.ViewModels;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Reservoom.Commands
{
    public class MakeReservationCommand : CommandBase
    {
        private readonly Hotel hotel;
        private readonly MakeReservationViewModel makeReservationViewModel;
        public MakeReservationCommand(MakeReservationViewModel makeReservationViewModel, Hotel hotel)
        {
            this.makeReservationViewModel = makeReservationViewModel;
            this.hotel = hotel;
            this.makeReservationViewModel.PropertyChanged += MakeReservationViewModel_PropertyChanged;
        }

        private void MakeReservationViewModel_PropertyChanged(object? sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(makeReservationViewModel.UserName))
                { 
                OnCanExecuteChanged();
            }
        }

        public override bool CanExecute(object? parameter)
        {

            return !string.IsNullOrEmpty(makeReservationViewModel.UserName) && base.CanExecute(parameter);
        }

        public override void Execute(object? parameter)
        {
            Reservation reservation = new Reservation(
                new RoomID(makeReservationViewModel.FloorNumber, makeReservationViewModel.RoomNumber),
                makeReservationViewModel.UserName,
                makeReservationViewModel.StartDate,
                makeReservationViewModel.EndDate);

            hotel.MakeReservation(reservation);


        }
    }
}
