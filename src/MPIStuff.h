/*
 *  MPIStuff.h
 *  GeneralGA
 *
 *  Created by Bill Sellers on 06/09/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

struct MPIRunSpecifier
{
    double startTime;
    int mpiSource;
    int clientIndex;
};

enum MPITag
{
    idle = 0,
    request_send_genome = 1,
    send_result = 2
};
