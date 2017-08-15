/* ESP8266Interface Example
 * Copyright (c) 2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ESP8266_H
#define ESP8266_H

#include "ATParser.h"
#include "ESP8266_TYPES.h"

/** ESP8266Interface class.
    This is an interface to a ESP8266 radio.
 */
class ESP8266
{
public:
    ESP8266(PinName tx, PinName rx, PinName reset, bool debug=false);

    /**
    * Check firmware version of ESP8266
    *
    * @return integer firmware version or -1 if firmware query command gives outdated response
    */
    int get_firmware_version(void);

    /**
    * Test AT command works with UART
    *
    * @return true if successful
    */
    bool test_ok();
    
    /**
    * Tests baudrate and set to FAST baud rate
    *
    * @return true if successful
    */
    bool autobaudrate_init();

    /**
    * Startup the ESP8266
    *
    * @param mode mode of WIFI 1-client, 2-host, 3-both
    * @return true only if ESP8266 was setup correctly
    */
    bool startup(int mode);

    /**
    * Hardware Reset ESP8266
    *
    * @return true only if ESP8266 hard resets successfully
    */
    bool hw_reset(void);

    /**
    * Software Reset ESP8266
    *
    * @return true only if ESP8266 soft resets successfully
    */
    bool sw_reset(void);

    /**
    * Configures UART speed of ESP8266
    * @note uses default 8 bit, no parity, 1 stop bit
    * @return true only if ESP8266 changes UART baud rate successfully
    */
    bool uart_configure(int baudrate);

    /**
    * Enable/Disable DHCP
    *
    * @param enabled DHCP enabled when true
    * @param mode mode of DHCP 0-softAP, 1-station, 2-both
    * @return true only if ESP8266 enables/disables DHCP successfully
    */
    bool dhcp(bool enabled, int mode);

    /**
    * Connect ESP8266 to AP
    *
    * @param ap the name of the AP
    * @param passPhrase the password of AP
    * @return true only if ESP8266 is connected successfully
    */
    bool connect(const char *ap, const char *passPhrase);

    /**
    * Disconnect ESP8266 from AP
    *
    * @return true only if ESP8266 is disconnected successfully
    */
    bool disconnect(void);

    /**
    * Get the IP address of ESP8266
    *
    * @return null-teriminated IP address or null if no IP address is assigned
    */
    const char *getIPAddress(void);

    /**
    * Get the MAC address of ESP8266
    *
    * @return null-terminated MAC address or null if no MAC address is assigned
    */
    const char *getMACAddress(void);

    /**
    * Set the MAC address of ESP8266
    *
    * @return true or false whether or not MAC address is assigned
    */
    bool setMACAddress(const char * mac_buffer);

     /** Get the local gateway
     *
     *  @return         Null-terminated representation of the local gateway
     *                  or null if no network mask has been recieved
     */
    const char *getGateway();

    /** Get the local network mask
     *
     *  @return         Null-terminated representation of the local network mask 
     *                  or null if no network mask has been recieved
     */
    const char *getNetmask();

    /* Return RSSI for active connection
     *
     * @return      Measured RSSI
     */
    int8_t getRSSI();

    /**
    * Check if ESP8266 is connected
    *
    * @return true only if the chip has an IP address
    */
    bool isConnected(void);

    /** Scan for available networks
     *
     * @param  ap    Pointer to allocated array to store discovered AP
     * @param  limit Size of allocated @a res array, or 0 to only count available AP
     * @return       Number of entries in @a res, or if @a count was 0 number of available networks, negative on error
     *               see @a nsapi_error
     */
    int scan(WiFiStationAccessPoint *res, unsigned limit);
    
    /**Perform a dns query
    *
    * @param name Hostname to resolve
    * @param ip   Buffer to store IP address
    * @return 0 true on success, false on failure
    */
    bool dns_lookup(const char *name, char *ip);

    /**Perform a ping
	* @param name Hostname or IP address
	* @return rtt_time_ms on success, -1 on failure
	*/
    int ping(const char *name);

    /**
    * Open a socketed connection
    *
    * @param type the type of socket to open "UDP" or "TCP"
    * @param id id to give the new socket, valid 0-4
    * @param port port to open connection with
    * @param addr the IP address of the destination
    * @return true only if socket opened successfully
    */
    bool open(const char *type, int id, const char* addr, int port);

    /**
    * Open a socketed connection
    *
    * @param type the type of socket to open "UDP" or "TCP"
    * @param id id to give the new socket, valid 0-4
    * @param remoteport port to open connection with
    * @param udplocalport optional port to listen with
    * @param udpmode optional port mode for peer restriction (see ESP_CONFIG.h)
    * @param remoteaddr the IP address of the destination
    * @return true only if socket opened successfully
    */
    bool open_f2(const char *type, int id, const char* remoteaddr, int remoteport, int udplocalport, int udpmode);

    /**
    * Sends data to an open socket
    *
    * @param id id of socket to send to
    * @param data data to be sent
    * @param amount amount of data to be sent - max 1024
    * @return true only if data sent successfully
    */
    bool send(int id, const void *data, uint32_t amount);

    /*
    * Sends data to an open socket to a remote
	*
	* @param id id of socket to send to
	* @param data data to be sent
	* @param amount amount of data to be sent - max 1024
	* @param remoteip address of the remote if different
	* @param port port of the remote if different
	* @return true only if data sent successfully
	*/
    bool sendto(int id, const void *data, uint32_t amount, const char *remoteip, int port);

    /**
    * Receives data from an open socket
    *
    * @param id id to receive from
    * @param ipv4_addr ip address who sent the packet
    * @param port port of ip who sent the packet
    * @param data placeholder for returned information
    * @param amount number of bytes to be received
    * @return the number of bytes received
    */
    int32_t recvfrom(int id, char *ipv4_addr, int *port, void *data, uint32_t amount);

    /**
    * Closes a socket
    *
    * @param id id of socket to close, valid only 0-4
    * @return true only if socket is closed successfully
    */
    bool close(int id);

    /**
    * Allows timeout to be changed between commands
    *
    * @param timeout_ms timeout of the connection
    */
    void setTimeout(uint32_t timeout_ms);

    /**
    * Checks if data is available
    */
    bool readable();

    /**
    * Checks if data can be written
    */
    bool writeable();

    /**
    * Attach a function to call whenever network state has changed
    *
    * @param func A pointer to a void function, or 0 to set as none
    */
    void attach(void (*func) (void));

    /**
    * Attach a function to call whenever network state has changed
    *
    * @param obj pointer to the object to call the member function on
    * @param method pointer to the member function to call
    * See MODSERIAL ATTACH FOR Example calls
    */
    template <typename T, typename M>
    void attach(T *obj, M method){
    	_serial.attach(obj,method);
    }

private:
    MODSERIAL _serial;
    ATParser _parser;
    DigitalOut _reset;
    bool dbg_on;

    struct packet {
        struct packet *next;
        int id;
        uint32_t len;
        char ipv4_rem[16];
        int port_rem;
        // data follows
    } *_packets, **_packets_end;
    void _packet_handler();
    bool recv_ap(wifi_station_ap_t *ap);

    char _ip_buffer[16];
    char _gateway_buffer[16];
    char _netmask_buffer[16];
    char _mac_buffer[18];
};

#endif
