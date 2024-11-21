using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
//using PlayGround.ViewModels;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace PlayGround
{
    public partial class MainWindowViewModel: ObservableObject
    {
        [ObservableProperty]
        [NotifyCanExecuteChangedFor(nameof(btChangeNameCommand))]
        private string firstName = "Weifeng";
        //public IRelayCommand ClickCMD { get; }

        //public MainWindowViewModel()
        //{

        //    ClickCMD = new RelayCommand(OnClickCMD, canExecute);
        //}


        //private bool canExecute()
        //  => FirstName == "Weifeng";


        [RelayCommand(IncludeCancelCommand = true)]

        //[RelayCommand]
        private async Task btChangeName(CancellationToken token)
        {
            try
            {
                await Task.Delay(5000, token);
                FirstName += "Wei";
            }
            catch (Exception)
            {

                throw;
            }
            
        }

        private  bool CanClick()
        {
            return FirstName == "Weifeng";
        }

        //private object CanGreetUser(string username)
        //{
        //    return string.IsNullOrWhiteSpace(username);

        //}
    }
}
