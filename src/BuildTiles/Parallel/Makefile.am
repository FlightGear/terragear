EXTRA_DIST=fgfs-launch-clients fgfs-launch-server genworklist.sh

bin_PROGRAMS = fgfs-tools-server fgfs-tools-client

fgfs_tools_server_SOURCES = server.cxx

fgfs_tools_server_LDADD = -lsgbucket -lsgmisc

fgfs_tools_client_SOURCES = client.cxx

fgfs_tools_client_LDADD = -lsgbucket -lsgmisc

INCLUDES += \
	-I$(top_builddir)/src \
	-I$(top_builddir)/src/Lib \
	-I$(top_builddir)/src/Construct
