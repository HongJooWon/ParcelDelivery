//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

//package cana;
import inet.mobility.contract.IMobility;

// added on July, 9 by harrison  from Bien Aime
// TODO auto-generated type
//


module Dest
{
 @display("i=misc/town");
    @class(inet::Dest);
	int missionID = default(0);
	
	submodules:
	    mobility: <default("StatNodeMob")> like IMobility {
            @display("p=20,10;is=s");
	    }  
}