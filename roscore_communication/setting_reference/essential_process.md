# first step
$ sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"

# second step
$ sudo gedit /etc/sysctl.conf

and add the following at the bottom

```
net.ipv4.icmp_echo_ignore_broadcasts=0
```

# third step

```
$ sudo service procps restart
```
