bin_PROGRAMS = fgfs-tools-server fgfs-tools-client

fgfs_tools_server_SOURCES = server.cxx

fgfs_tools_server_LDADD = -lsgbucket -lsgmisc

fgfs_tools_client_SOURCES = client.cxx

fgfs_tools_client_LDADD = -lsgbucket -lsgmisc

INCLUDES += \
	-I$(top_builddir) \
	-I$(top_builddir)/Lib \
	-I$(top_builddir)/Construct
