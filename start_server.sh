#!/bin/bash
sudo ./build/bin/Server |& tee -a server_output_log.txt

# To copy a file from B to A while logged into B:
# scp /path/to/file username@a:/path/to/destination
# To copy a file from B to A while logged into A:
# scp username@b:/path/to/file /path/to/destination


# rm 0 size file
# find /tmp -size  0 -print0 |xargs -0 rm
