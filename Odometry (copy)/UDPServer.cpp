#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <string>
#include <fcntl.h>

#define earthRadiusKm 6371.0
#define BUFLEN 512
using namespace std;

double deg2rad(double deg) {
    return (deg * M_PI / 180);
}

double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
    double lat1r, lon1r, lat2r, lon2r, u, v;
    lat1r = deg2rad(lat1d);
    lon1r = deg2rad(lon1d);
    lat2r = deg2rad(lat2d);
    lon2r = deg2rad(lon2d);
    u = sin((lat2r - lat1r) / 2);
    v = sin((lon2r - lon1r) / 2);
    return 2.0 * earthRadiusKm *
           asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

vector<string> split(const string &str, const string &delim) {
    vector<string> tokens;
    size_t prev = 0, pos = 0;
    do {
        pos = str.find(delim, prev);
        if (pos == string::npos) pos = str.length();
        string token = str.substr(prev, pos - prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    } while (pos < str.length() && prev < str.length());
    return tokens;
}

int main(int argc, char *argv[]) {
    int PORT;
    if (argc < 2) {
        std::cout << "./UDPServer <port>\n";
        return -1;
    }
    PORT = atoi(argv[1]);
    std::cout << PORT << std::endl;

    // FD Sets
    fd_set readFds;
    fd_set tempFds;
    FD_ZERO(&readFds);
    FD_ZERO(&tempFds);

    // UDP socket
    struct sockaddr_in serverAddr, otherAddr;
    int s;
    ssize_t recvLen;
    socklen_t slen = sizeof(otherAddr);

    char buf[BUFLEN];
    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        std::cout << "Error creating socket\n";
        return -1;
    }

    fcntl(s, F_SETFL, O_NONBLOCK);

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (sockaddr *) &serverAddr, sizeof(serverAddr)) == -1) {
        std::cout << "Bind error\n";
        return -1;
    }

    listen(s, 1);
    FD_SET(s, &readFds);

    int fdmax = s;

    std::string delimiter = ",";
    size_t start = 0;
    size_t end = 0;
    std::string token;
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    while (true) {
        tempFds = readFds;
        if (select(fdmax + 1, &tempFds, NULL, NULL, NULL) == -1) {
            std::cout << "Select error\n";
            fflush(stdout);
            return -1;
        }
        if (FD_ISSET(s, &tempFds)) {
            std::cout << "Got data\n";
            fflush(stdout);
            // Get data from UDP socket
            memset(buf, 0, sizeof(buf));

            recvLen = recvfrom(s, buf, BUFLEN, 0, (sockaddr *)
                                       &otherAddr,
                               &slen);
            std::cout << buf;
            std::string bufString(buf);
            vector<string> fields = split(bufString, delimiter);
            if (fields[0] == "O") {
                std::cout << "Orientation\n";
                double x = std::stod(fields[3]);
                double y = std::stod(fields[4]);
                double z = std::stod(fields[5]);
                std::cout << x << " " << y << " " << z << std::endl;
            } else if (fields[0] == "G") {
                std::cout << "GPS\n";
                double lat = std::stod(fields[4]);
                double lon = std::stod(fields[5]);
                std::cout << lat << " " << lon << std::endl;
            }

        }

    }


    return 0;
}