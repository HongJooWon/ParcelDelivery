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
package cana;
import inet.examples.mobility.DroneHost;
import inet.node.inet.AdhocHost;
//import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.networklayer.ipv4.RoutingTableRecorder;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.examples.mobility.Destination;


network ParcelDeliveryNetwork
{
    parameters:
        int numdrones;
        @display("bgb=600,400");
        @statistic[totalDistance](source = tspDistance; record = vector;);
        @statistic[totalTime](source = missionTime; record = vector;); 
    submodules:
        drone[numdrones]: DroneHost {
            parameters:
                @display("p=300,300;r=,,#707070");
        }
		radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=550,25");
        }
        routingTableRecorder: RoutingTableRecorder {
            parameters:
                @display("p=550,100");
        }
   		configurator: Ipv4NetworkConfigurator {
            config = xml("<config><interface hosts='*' address='192.168.0.x' netmask='255.255.255.0'/></config>");
            @display("p=550,300");
        }
		gcs: AdhocHost {
            @display("i=device/antennatower_vl;r=,,#707070;p=550,200");
        }
//        dest[ndst]: Destination{
//            parameters:
//            	@display("p=91,370;r=,,#707070");
//        }
}