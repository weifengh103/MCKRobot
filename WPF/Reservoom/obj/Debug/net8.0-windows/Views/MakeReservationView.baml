   M S B A M L   `   `   `  ����D  @Reservoom, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null1clr-namespace:Reservoom.ViewsReservoom.Views  5      )    #Reservoom.Views.MakeReservationView   5      W SPresentationCore, Version=8.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=8.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35e aSystem.Windows.Controls.Ribbon, Version=8.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089R NWindowsBase, Version=8.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35F 9http://schemas.microsoft.com/winfx/2006/xaml/presentation     5      R NSystem.Xaml, Version=8.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5      Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     5      @d2http://schemas.microsoft.com/expression/blend/2008     5      /localclr-namespace:Reservoom.Views     5      .��5	      �   � RowDefinitions  5
   
   �� 5      $
��auto �6   �� 5      $
��auto �6   �� 5      $
��auto �6   �� 5      $
��auto �6   �� 5      $
��auto �6   .+�5   
   �� $��0��5      $��24(�5      $��Make REservation��5      � 5   
   $��1��6     5      �� 5      $
��auto �6    �� 5      $
��auto �6    .+�5      �� $��0��6   $
��Name��6%   �� 5      $��1��6   $��0 5 0 0q�6c   ��6#   �� *
UserName+ �� UpdateSourceTrigger$ PropertyChanged=�6c   5      � 5    
   $��2��6   $��0 5 0 20q�6     5!      �� 5"      $
��auto �6    �� 5#      $
��auto �6     � ColumnDefinitions 5%      �� 5&      $
��auto �6#   �� 5'      $
��auto �6#   .+�5)      �� $��0��6   $��0��6%   $��	Floor No.��65   �� 5*      $��1��6   $��0��6#   $��0 5 0 0q�6v   ��63   �� *FloorNumber+$ PropertyChanged=�6v   �� 5+      $��0��6   $��1��6%   $��Room No.��65   �� 5,      $��1��6   $��1��6#   $��0 5 0 0q�6u   ��63   �� *
RoomNumber+$ PropertyChanged=�6u   5-      � 5/   
   $��3��6   $��0 5 0 20q�6     50      �� 51      $
��auto �6    �� 52      $
��auto �6     54      �� 55      $
��auto �6#   �� 56      $
��auto �6#   .+�58      �� $��0��6   $��0��6%   $��
Start Date��65   (  "System.Windows.Controls.DatePicker  59      $��1��6   $��0��6&   $��0 5 0 0q�6      SelectedDate 66   �� *	StartDate+$ PropertyChanged=�6   �� 5:      $��0��6   $��1��6%   $��End Date��65     5;      $��1��6   $��1��6&   $��0 5 0 0q�6}    66   �� *	EndDate+$ PropertyChanged=�6}   5<      �� 5>   
   $��4��6    �� Orientation$ 
Horizontal=�6"   $��0 20 0 0q�6;   .+�5?      �� $��Submit��6   ��6&   �� *SubmitCommand+�� 5@      $��Cancel��6   $��5 0 0 0 q�6&   ��68   �� *CancelCommand+5A      5E      5I      