using Reservoom.Commands;
using Reservoom.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace Reservoom.ViewModels
{
    public class MakeReservationViewModel: ViewModelBase
    {
		private string userName;
		public string UserName
		{
			get
			{
				return userName;
			}
			set
			{
				userName = value;
				OnPropertyChanged(nameof(UserName));
			}
		}

		private int roomNumber;
		public int RoomNumber
		{
			get
			{
				return roomNumber;
			}
			set
			{
				roomNumber = value;
				OnPropertyChanged(nameof(RoomNumber));
			}
		}

		private int floorNumber;
		public int FloorNumber
		{
			get
			{
				return floorNumber;
			}
			set
			{
				floorNumber = value;
				OnPropertyChanged(nameof(FloorNumber));
			}
		}

		private DateTime startDate;
		public DateTime StartDate
		{
			get
			{
				return startDate;
			}
			set
			{
				startDate = value;
				OnPropertyChanged(nameof(StartDate));
			}
		}

		private DateTime endDate;
		public DateTime EndDate
		{
			get
			{
				return endDate;
			}
			set
			{
				endDate = value;
				OnPropertyChanged(nameof(EndDate));
			}
		}

        public ICommand SubmitCommand { get; }
        public ICommand CancelCommand { get; }

		public MakeReservationViewModel(Hotel hotel)
		{
			SubmitCommand = new MakeReservationCommand(this,hotel);
		}
	}
}
