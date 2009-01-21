EXTRA_DIST=fgfs-launch-clients fgfs-launch-server genworklist.sh

bin_PROGRAMS = fgfs-tools-server fgfs-tools-client

fgfs_tools_server_SOURCES = server.cxx

fgfs_tools_server_LDADD = -lsgbucket -lsgmisc -lsgdebug -lsgxml $(network_LIBS)

fgfs_tools_client_SOURCES = client.cxx

fgfs_tools_client_LDADD = -lsgbucket -lsgmisc -lsgdebug -lsgxml $(network_LIBS) -lplibul

INCLUDES = \
	-I$(top_srcdir)/src \
	-I$(top_srcdir)/src/Lib \
	-I$(top_srcdir)/src/BuildTiles
