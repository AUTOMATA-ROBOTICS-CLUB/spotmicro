// Linux client: reads the keyboard (or a DualSense) from /dev/input and sends
// every key press to the robot over UDP as a 4-byte big-endian evdev key code
// (see wifi.h on the ESP32 side for what each key does).
//
// Build: g++ -std=c++17 -Wall -o pc_controller pc_controller.cpp
// Run:   join the robot's WiFi AP, then sudo ./pc_controller (or add
//        yourself to the input group to drop the sudo). With the keyboard,
//        Esc or Ctrl+C quits.

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/input.h>
#include <map>
#include <netinet/in.h>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

// Input device: set one to 1
#define IS_KEYBOARD 1
#define IS_DUALSENSE 0
#define IS_XBOX 0
// ESP32 UDP server port
#define PORT 4210
// ESP32 WiFi access point IP
#define IP "192.168.4.1"

#if IS_DUALSENSE
#define PATH "Sony Interactive Entertainment DualSense Wireless Controller"
#endif

// if this is one set currentPress to the correct event key and send over udp
std::atomic<int> buttonIsPressed{0};
std::atomic<int> currentPress{0};

// TODO: Add 2 seperate threads, one for udp client and one for processing
// inputs

std::map<int, std::string_view> btnMap = {
    {304, "Cross"}, {305, "Circle"}, {307, "Triangle"}, {308, "Square"}};

void getEvent(int event) {
  if (auto it = btnMap.find(event); it != btnMap.end()) {
    std::cout << it->second << "\n";
  }
}

// True if the device is a real keyboard: it has the A, Z and Esc keys. (The
// power button, lid switch and hotkeys also show up as keyboards.)
bool isKeyboard(int fd) {
  // One bit per key code, set if the device has that key.
  unsigned long keys[KEY_MAX / (8 * sizeof(long)) + 1] = {};
  if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keys)), keys) < 0)
    return false;
  auto has = [&](int key) {
    return (keys[key / (8 * sizeof(long))] >> (key % (8 * sizeof(long)))) & 1;
  };
  return has(KEY_A) && has(KEY_Z) && has(KEY_ESC);
}

// Keys the robot knows (onKey() in wifi.h).
void printKeyboardHelp(void) {
  std::cout << "\nKeyboard controls (same letters as the calibrate "
               "commands):\n"
               "  left / right     previous / next joint\n"
               "  + / -  up / down nudge centre +1 / -1 tick\n"
               "  pgup / pgdn      nudge centre +10 / -10 ticks\n"
               "  a                select all joints\n"
               "  h / z            stand / rest\n"
               "  d                flip direction\n"
               "  l                list the table\n"
               "  w / r            write to / reload from flash\n"
               "  x                reset to the defaults in robot_config.h\n"
               "  o                outputs off (servos limp)\n"
               "  ?                help (on the robot's serial monitor)\n"
               "  esc / ctrl+c     quit\n"
               "g, k, t and m need a number: use the serial monitor.\n\n";
}

// Returns the N of the /dev/input/eventN that is the first keyboard (or with
// IS_DUALSENSE, whose name is PATH), or -1.
int FindDevice(void) {
  std::cout << "looking for device"
            << "\n";

  for (int i = 0; i < 64; i++) {
    char path[64];
    snprintf(path, sizeof(path), "/dev/input/event%d", i);
    int fd = open(path, O_RDONLY);
    if (fd < 0)
      continue;

    char name[256] = {};
    ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name);
#if IS_KEYBOARD
    bool found = isKeyboard(fd);
#else
    bool found = strcmp(name, PATH) == 0;
#endif
    close(fd);

    if (found) {
      std::cout << "Found " << path << " (" << name << ")\n";
      return i;
    }
  }

  std::cout << "Device was not found (is it connected, and can you read "
               "/dev/input? try sudo)"
            << "\n";
  return -1;
}

// Opens the controller for blocking reads, so the loops sleep until there is
// an event. Returns the file descriptor, or -1.
int openController(void) {
  int i = FindDevice();
  if (i < 0)
    return -1;

  char path[64];
  snprintf(path, sizeof(path), "/dev/input/event%d", i);
  int fd = open(path, O_RDONLY);
  if (fd < 0)
    std::cout << "Error opening " << path << ": " << strerror(errno) << "\n";
  return fd;
}

// Create UDP Client for esp32 wifi connection
int createClient(void) {
  int fd = openController();
  if (fd < 0)
    return 1;

  int s = socket(AF_INET, SOCK_DGRAM, 0);
  if (s < 0) {
    std::cout << "Could not create socket: " << strerror(errno) << "\n";
    close(fd);
    return 1;
  }

  struct sockaddr_in server_addr = {};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);
  if (inet_pton(AF_INET, IP, &server_addr.sin_addr) <= 0) {
    std::cout << "Wrong address"
              << "\n";
    close(s);
    close(fd);
    return 1;
  }

#if IS_KEYBOARD
  // Wait for the Enter that started the program to be released, then take
  // the keyboard for this program only, so the keys don't also type into the
  // terminal. It is given back when the program exits.
  usleep(500000);
  if (ioctl(fd, EVIOCGRAB, 1) < 0)
    std::cout << "Could not grab the keyboard: " << strerror(errno) << "\n";
  printKeyboardHelp();
  bool ctrlDown = false;
#endif

  int rc = 0;
  struct input_event event;
  while (true) {
    ssize_t n = read(fd, &event, sizeof(event));
    // The controller was unplugged or turned off.
    if (n < 0 && errno != EINTR) {
      std::cout << "Controller disconnected: " << strerror(errno) << "\n";
      rc = 1;
      break;
    }
    if (n != sizeof(event))
      continue;

#if IS_KEYBOARD
    // Remember whether Ctrl is held (value 0 = released), for Ctrl+C.
    if (event.type == EV_KEY &&
        (event.code == KEY_LEFTCTRL || event.code == KEY_RIGHTCTRL))
      ctrlDown = event.value != 0;
#endif

    if (event.type == EV_KEY && event.value == 1) {
#if IS_KEYBOARD
      // Esc or Ctrl+C quits (the terminal can't see Ctrl+C while the
      // keyboard is grabbed).
      if (event.code == KEY_ESC || (event.code == KEY_C && ctrlDown))
        break;
#endif
      uint32_t packet = htonl(event.code);
      getEvent(event.code);
      std::cout << "Event Code: " << event.code << "\n";
      if (sendto(s, &packet, sizeof(packet), 0, (struct sockaddr *)&server_addr,
                 sizeof(server_addr)) < 0)
        std::cout << "Send failed (joined the robot's WiFi?): "
                  << strerror(errno) << "\n";
    }

    else if (event.type == EV_ABS) {
      if (event.code == ABS_HAT0X) {
        if (event.value == 1) {
          std::cout << "Right"
                    << "\n";
        } else if (event.value == -1) {
          std::cout << "Left"
                    << "\n";
        }
      } else if (event.code == ABS_HAT0Y) {
        if (event.value == 1) {
          std::cout << "Down"
                    << "\n";
        } else if (event.value == -1) {
          std::cout << "Up"
                    << "\n";
        }
      }
    }
  }

  close(fd);
  close(s);
  return rc;
}

// Keep until i add different threads for the udp client and this function
void controller_loop(void) {
  int fd = openController();
  if (fd < 0)
    return;

  struct input_event event;
  while (read(fd, &event, sizeof(event)) == sizeof(event)) {
    if (event.type == EV_KEY) {
      getEvent(event.code);
    }
    // else if(event.type == EV_ABS) {
    // std::cout << "Joystick event" << "\n";
    //}
  }
  close(fd);
}

int main(void) { return createClient(); }
