using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WHPG2.Models
{
    public class Pump
    {
        public Pump(int id, string name)
        {
            this.id = id;
            this.name = name;
        }

	
		private CancellationTokenSource cts;
        private Task runTask;

        private int id;

		public int ID
		{
			get { return id; }
			set { id = value; }
		}

		private string name;

        public string Name
		{
			get { return name; }
			set { name = value; }
		}

		private int rpm;

		public int RPM
		{
			get { return rpm; }
			set { rpm = value; }
		}

		public event EventHandler<string> StatusChanged;


        public void StartPump()
        {
            // Reinitialize the cancellation token source if null or canceled
            if (cts == null || cts.IsCancellationRequested)
            {
                cts = new CancellationTokenSource();
            }

            runTask = Task.Run(async () =>
            {
                while (!cts.IsCancellationRequested)
                {
                    try
                    {
                        await Task.Delay(500000, cts.Token);  // Use async delay with cancellation
                        rpm++;
                        OnStatusChanged($"RPM: {rpm}");  // Update status with RPM
                    }
                    catch (TaskCanceledException)
                    {
                        OnStatusChanged("Pump Stop");
                        break;  // Exit the loop if the task is canceled
                    }
                }
            });

        }

        public void StopPump()
        {
            if (cts != null)
            {
                cts.Cancel();
            }
        }

        private void OnStatusChanged(string status)
		{
			StatusChanged?.Invoke(this, status);
		}

	 

	}
}
