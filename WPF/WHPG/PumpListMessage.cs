using CommunityToolkit.Mvvm.Messaging.Messages;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using WHPG.Models;

namespace WHPG
{
    public class PumpListMessage : ValueChangedMessage<AllPumpsList>
    {
        public PumpListMessage(AllPumpsList value) : base(value)
        {
        }
    }
}
