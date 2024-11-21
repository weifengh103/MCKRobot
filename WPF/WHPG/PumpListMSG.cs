using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CommunityToolkit.Mvvm.Messaging;
using CommunityToolkit.Mvvm.Messaging.Messages;
using WHPG.Models;

namespace WHPG
{
    public class PumpListMSG : ValueChangedMessage<AllPumpsList>
    {
        public PumpListMSG(AllPumpsList value) : base(value)
        {
        }
    }
}
