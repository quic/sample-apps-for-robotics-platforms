##################### SIMPLE HTTP and WEB SOCKET SERVER on RB5 ###################

1. Install Packages

   sh-4.4# apt remove --purge nodejs npm
   sh-4.4# apt clean
   sh-4.4# apt autoclean
   sh-4.4# apt install -f
   sh-4.4# apt autoremove

# Using Debian, as root
# Find the latest version at https://github.com/nodesource/distributions#debinstall Latest version 16.x now

2. Install npm and nodejs

   sh-4.4# curl -fsSL https://deb.nodesource.com/setup_16.x | bash -
   sh-4.4# apt-get install -y nodejs
   sh-4.4# curl -sL https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add -
   sh-4.4# apt-get update && sudo apt-get install yarn

3. Check the version 

   sh-4.4# npm -version
   7.20.3
   sh-4.4# nodejs -v
   v16.6.2 

4. Push the binaries
   C:\>adb push ~/simple_http_ws_server/server.js /data/node/
   C:\>adb push ~/simple_http_ws_server/server.html /data/node/
   C:\>adb shell sync
   C:\>adb shell
   sh-4.4# node server.js
   connection received 
   received: hello world