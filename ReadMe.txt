	Efficient Parcel Delivery Service (EPDS)
	________________________________________
	
	Versions:
	________
	
	OMNeT++ version 5.6.2
	INET    version Use the version provided with the EfficientParcelDelivery source codes.
		
	Import inet and EfficientParcelDelivery in your simulator!
	
	Configuration:
	______________	
	(1) Linking INET to EfficientParcelDelivery
	Right Click CANA >>> properties  >>> Project References >>>  tick inet >>>   Apply and close.
	
	(2)In omnetpp.ini, modify the parcelSelecitionMethod as follows:
	parcelSelecitionMethod = 0  to run  CDPF        //Closest-Deadline-Parcel-First
	parcelSelecitionMethod = 1  to run  CNPF       //Closest-Neighbor-Parcel-First
	parcelSelecitionMethod = 2  to run  EPDS      //Efficient Parcel Delivery Service distance/weight
	parcelSelecitionMethod = 3  to run  RSPF     //Randomly-Selected-Parcel-First
	parcelSelecitionMethod = 4  to run  HPF     //Heaviest Parcel First	
		
		
	Enjoy our Drone Network Simulation!