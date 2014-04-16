/*
  Copyright (C) 2012 CrossWing Inc. www.crosswing.com
  All Rights Reserved.
*/

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include <nav2_driver/nav2remote.h>

Nav2Remote::Nav2Remote( const char *host, int port)
    : line(NULL), lineLen(0), fd(-1)
{
    if(port < 1 || port > 65535) throw std::invalid_argument("Invalid port");

    char service[6];
    sprintf(service, "%d", port);

    struct addrinfo *ai;
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    int s = getaddrinfo( host, service, &hints, &ai);
    if( s != 0) throw std::runtime_error("Can't get address info");

    for( struct addrinfo* rp = ai;; rp = rp->ai_next) {
        if( rp == NULL) {
            freeaddrinfo( ai);
            throw std::runtime_error("Can't connect to robot");
        }

        fd = socket( rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if( fd == -1) continue;
        if( connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) break;
        close(fd);
        fd = -1;
    }

    freeaddrinfo(ai);
}

Nav2Remote::~Nav2Remote()
{
    if(line) free(line);
    close(fd);
}

int Nav2Remote::readLine() const
{
    for( int pos=0;; ++pos) {
        if( lineLen <= pos+1) {
            lineLen += 32;
            line = (char*)realloc(line, lineLen);
            if( !line) throw std::bad_alloc();
        }
        if( read(fd, &line[pos], 1) <= 0) return -1;

        if( line[pos] == '\r') {
            --pos;  // Ignore carriage returns, just in case!
            continue;
        }

        if( line[pos] == '\n') {
            line[pos] = 0;

            // Ignore lines that begin with | or +.
            if( line[0] == '|' || line[0] == '+') {
                pos = -1;
                continue;
            }

            return pos;
        }
    }
}

int Nav2Remote::setTargetOrientation( double orientation)
{
    char msg[128];
    int p = sprintf(msg, "o %lf\n", orientation);
    return write(fd,msg,p) == p ? 0 : -1;
}

int Nav2Remote::setAbsoluteVelocity( double vx, double vy)
{
    char msg[128];
    int p = sprintf(msg, "av %lf %lf\n", vx, vy);
    return write(fd,msg,p) == p ? 0 : -1;
}

int Nav2Remote::setRelativeVelocity( double vx, double vy, double turnRate)
{
    char msg[128];
    int p = sprintf(msg, "v %lf %lf %lf\n", vx, vy, turnRate * (180.0 / M_PI));
    return write(fd,msg,p) == p ? 0 : -1;
}

int Nav2Remote::estimatePosition(
    double& x, double& y, double& orientation) const
{
    if( write(fd, "q\n", 2) != 2) return -1;

    // Read the result
    if( readLine() < 0) return -1;

    int qlen;
    sscanf(line, "%lf %lf %lf %d", &x, &y, &orientation, &qlen);
    orientation *= (M_PI / 180.0);

    return 0;
}

int Nav2Remote::setPosition( double x, double y, double orientation)
{
    char msg[128];
    int p = sprintf(msg, "p %lf %lf %lf\n", x, y, orientation * (180.0 / M_PI));
    return write(fd,msg,p) == p ? 0 : -1;
}

int Nav2Remote::stop()
{
    return write(fd,"s\n",2) == 2 ? 0 : -1;
}

int Nav2Remote::turnLeft( double angle)
{
    char msg[128];
    int p = sprintf(msg, "lt %lf\n", angle * (180.0 / M_PI));
    return write(fd,msg,p) == p ? 0 : -1;
}

int Nav2Remote::move( double dist, double direction)
{
    char msg[128];
    int p = sprintf(msg, "mv %lf %lf\n", dist, direction * (180.0 / M_PI));
    return write(fd,msg,p) == p ? 0 : -1;
}

int Nav2Remote::setMaxSpeed( double maxSpeed)
{
    char msg[128];
    int p = sprintf(msg, "sms %lf\n", maxSpeed);
    return write(fd,msg,p) == p ? 0 : -1;
}

int Nav2Remote::setMaxAccel( double maxAccel)
{
    char msg[128];
    int p = sprintf(msg, "sma %lf\n", maxAccel);
    return write(fd,msg,p) == p ? 0 : -1;
}

int Nav2Remote::setMaxCorneringError( double maxCorneringError)
{
    char msg[128];
    int p = sprintf(msg, "smce %lf\n", maxCorneringError);
    return write(fd,msg,p) == p ? 0 : -1;
}

double Nav2Remote::getMaxSpeed() const
{
    if( write(fd, "qms\n", 4) != 4) return -1;

    // Read the result
    if( readLine() < 0) return -1;

    double result;
    sscanf(line, "%lf", &result);

    return result;
}

double Nav2Remote::getMaxAccel() const
{
    if( write(fd, "qma\n", 4) != 4) return -1;

    // Read the result
    if( readLine() < 0) return -1;

    double result;
    sscanf(line, "%lf", &result);

    return result;
}

double Nav2Remote::getMaxCorneringError() const
{
    if( write(fd, "qmce\n", 5) != 5) return -1;

    // Read the result
    if( readLine() < 0) return -1;

    double result;
    sscanf(line, "%lf", &result);

    return result;
}

int Nav2Remote::getQueueSize() const
{
    if( write(fd, "q\n", 2) != 2) return -1;

    // Read the result
    if( readLine() < 0) return -1;

    double x;
    int qlen;
    sscanf(line, "%lf %lf %lf %d", &x, &x, &x, &qlen);

    return qlen;
}

int Nav2Remote::wait() const
{
    while(1) {
        int rc = getQueueSize();
        if( rc < 1) return rc;
        usleep(100000);
    }
}
