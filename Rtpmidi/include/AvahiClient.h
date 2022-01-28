//
// Created by Raghavasimhan Sankaranarayanan on 4/9/21. Ref from https://www.avahi.org/doxygen/html/client-publish-service_8c-example.html
//

#ifndef AVAHI_CLIENT_H
#define AVAHI_CLIENT_H

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <avahi-client/client.h>
#include <avahi-client/publish.h>
#include <avahi-common/alternative.h>
#include <avahi-common/simple-watch.h>
#include <avahi-common/malloc.h>
#include <avahi-common/error.h>


static AvahiEntryGroup *group = nullptr;
static AvahiSimplePoll *simple_poll = nullptr;
static char *name = nullptr;
static void create_services(AvahiClient *c);

static void entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state, AVAHI_GCC_UNUSED void *userdata) {
    assert(g == group || group == nullptr);
    group = g;
    /* Called whenever the entry group state changes */
    switch (state) {
        case AVAHI_ENTRY_GROUP_ESTABLISHED :
            /* The entry group has been established successfully */
            fprintf(stderr, "Service '%s' successfully established.\n", name);
            break;
        case AVAHI_ENTRY_GROUP_COLLISION : {
            char *n;
            /* A service name collision with a remote service
             * happened. Let's pick a new name */
            n = avahi_alternative_service_name(name);
            avahi_free(name);
            name = n;
            fprintf(stderr, "Service name collision, renaming service to '%s'\n", name);
            /* And recreate the services */
            create_services(avahi_entry_group_get_client(g));
            break;
        }
        case AVAHI_ENTRY_GROUP_FAILURE :
            fprintf(stderr, "Entry group failure: %s\n", avahi_strerror(avahi_client_errno(avahi_entry_group_get_client(g))));
            /* Some kind of failure happened while we were registering our services */
            avahi_simple_poll_quit(simple_poll);
            break;
        case AVAHI_ENTRY_GROUP_UNCOMMITED:
        case AVAHI_ENTRY_GROUP_REGISTERING:
            break;
    }
}

static void create_services(AvahiClient *c) {
    char *n, r[128];
    int ret;
    assert(c);
    /* If this is the first time we're called, let's create a new
     * entry group if necessary */
    if (!group)
        if (!(group = avahi_entry_group_new(c, entry_group_callback, nullptr))) {
            fprintf(stderr, "avahi_entry_group_new() failed: %s\n", avahi_strerror(avahi_client_errno(c)));
            goto fail;
        }
    /* If the group is empty (either because it was just created, or
     * because it was reset previously, add our entries.  */
    if (avahi_entry_group_is_empty(group)) {
        fprintf(stderr, "Adding service '%s'\n", name);
        /* Create some random TXT data */
        snprintf(r, sizeof(r), "random=%i", rand());
        /* We will now add two services and one subtype to the entry
         * group. The two services have the same name, but differ in
         * the service type (IPP vs. BSD LPR). Only services with the
         * same name should be put in the same entry group. */
        /* Add the service for IPP */

        if ((ret = avahi_entry_group_add_service(group, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC, (AvahiPublishFlags) 0, name, "_apple-midi._udp", nullptr,
                                                 nullptr, 5004, "test=blah", r, NULL)) < 0) {
            if (ret == AVAHI_ERR_COLLISION)
                goto collision;
            fprintf(stderr, "Failed to add _ipp._tcp service: %s\n", avahi_strerror(ret));
            goto fail;
        }

        /* Tell the server to register the service */
        if ((ret = avahi_entry_group_commit(group)) < 0) {
            fprintf(stderr, "Failed to commit entry group: %s\n", avahi_strerror(ret));
            goto fail;
        }
    }
    return;
    collision:
    /* A service name collision with a local service happened. Let's
     * pick a new name */
    n = avahi_alternative_service_name(name);
    avahi_free(name);
    name = n;
    fprintf(stderr, "Service name collision, renaming service to '%s'\n", name);
    avahi_entry_group_reset(group);
    create_services(c);
    return;
    fail:
    avahi_simple_poll_quit(simple_poll);
}

#endif // AVAHI_CLIENT_H