import config
import network
import machine
import socket

class server:
    
    def config_page(self, t_custom_name, t_ssid, mqtt_server, mqtt_port, mqtt_user, mqtt_pass):
        if t_ssid == None:
            t_ssid = '-'
        if t_custom_name == None:
            t_custom_name = '-'
        if mqtt_server == None:
            mqtt_server = '-'
        if mqtt_user == None:
            mqtt_user = '-'
        if mqtt_port == None:
            mqtt_port = '-'
        if mqtt_pass == None:
            mqtt_pass = '-'
        html = """<html><head> <title>Module Webserver</title> <meta name="viewport" content="width=device-width, initial-scale=1">
            <link rel="icon" href="data:,"> <style>html{font-family: Helvetica; display:inline-block; margin: 0px auto; text-align: center;}
            h1{color: #0F3376; padding: 2vh;}p{font-size: 1.5rem;}.button{display: inline-block; background-color: #e7bd3b; border: none; 
            border-radius: 4px; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
            .button2{background-color: #4286f4;}</style></head><body> <h1>Local Config</h1> 
            <h2>Wifi config</h2>
            <form action="/?save">
            <p>SSID: <input type="text" name="ssid" value=""" + "\"" + t_ssid + "\"" + """><br></p>
            <p>Password: <input type="text" name="password"><br></p>
            <input type="submit" value="Guardar" class="button">
            </form>
            <h2>Custom name</h2>
            <form action="/?save">
            <p>Device Name: <input type="text" name="custom_name" value=""" + "\"" + t_custom_name + "\"" + """><br></p>
            <input type="submit" value="Guardar" class="button">
            </form>
            <h2>MQTT Config</h2>
            <form action="/?save">
            <p>MQTT Server: <input type="text" name="mqtt_server" value=""" + "\"" + mqtt_server + "\"" + """><br></p>
            <p>MQTT Port: <input type="text" name="mqtt_port" value=""" + "\"" + mqtt_port + "\"" + """><br></p>
            <p>MQTT User: <input type="text" name="mqtt_user" value=""" + "\"" + mqtt_user + "\"" + """><br></p>
            <p>MQTT Pass: <input type="text" name="mqtt_pass" value=""" + "\"" + mqtt_pass + "\"" + """><br></p>
            <input type="submit" value="Guardar" class="button">
            </form>
            <a href="/?resetear"><button class="button">Resetear</button></a>
            <p><h3 align='right'>Version: """ + str(locals()['VERSION']) + """</h3></p>
            </body></html>"""
        return html

    def __init__(self):
        self.ap = network.WLAN(network.AP_IF)
        self.ap.active(True)
        self.ap.config(authmode=3, password='abcd1234')
        self.led_green = machine.Pin(17, machine.Pin.OUT)
        self.led_green.off()
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind(('', 80))
        self.s.listen(5)
        self.s.settimeout(1)
        self.config_handler = config.local()
        self.t_ssid, self.t_pass = self.config_handler.getSettings([ 'ssid', 'password' ])
        self.t_custom_name, = self.config_handler.getSettings([ 'custom_name' ])
        self.mqtt_server, self.mqtt_port, self.mqtt_user, self.mqtt_pass = self.config_handler.getSettings(['mqtt_server','mqtt_port','mqtt_user','mqtt_pass'])

    def stop(self):
        self.ap.active(False)

    def run(self):
        try:
            conn, addr = self.s.accept()
            print('Got a connection from %s' % str(addr))
            request = conn.recv(1024)
            request = str(request)
            print('Content = %s' % request)
            first_space = request.find(' ')
            second_space = request.find(' ', first_space + 1)
            reset_order = request.find('/?resetear')
            get_cmd = request[first_space:second_space]
            if get_cmd.startswith(' /?'):
                print('GET CMD: ' + get_cmd)
                t_cmd = get_cmd.replace('/?','')
                t_cmd = t_cmd.replace(' ','')
                t_cmd = t_cmd.split('&')
                print('Parsed msg: ' + str(t_cmd))
                if len(t_cmd) > 1:
                    for cmd in t_cmd:
                        print('Cmd to parse: ' + cmd)
                        varname,value = cmd.split('=')
                        if value == '-' or value == '':
                            pass
                        else:
                            self.config_handler.saveSetting(varname, value)
            if reset_order == 6:
                print('Resetting device')
                machine.reset()
            response = self.config_page(self.t_custom_name, self.t_ssid, self.mqtt_server, self.mqtt_port, self.mqtt_user, self.mqtt_pass)
            conn.send('HTTP/1.1 200 OK\n')
            conn.send('Content-Type: text/html\n')
            conn.send('Connection: close\n\n')
            conn.sendall(response)
            conn.close()
        except OSError:
            #print('Timedout')
            pass
        except Exception as e:
            print('Generic Exception: ' + str(e))