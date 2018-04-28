/*
 *  Geom.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Wrapper class to hold ODE geom

#ifndef Geom_h
#define Geom_h

#include <vector>

#include "NamedObject.h"

class Contact;

class Geom: public NamedObject
{
public:

    Geom();
    virtual ~Geom();

    enum GeomLocation
    {
        environment,
        body
    };

    void SetBody(dBodyID body);
    dBodyID GetBody();
    dGeomID GetGeomID() { return m_GeomID; }

    // these functions set the geom position relative to its body
    void SetPosition (dReal x, dReal y, dReal z);
    void SetQuaternion(dReal q0, dReal q1, dReal q2, dReal q3);
    void SetPosition (const char *buf);
    void SetQuaternion(const char *buf);

    // return body local values
    const dReal *GetPosition();
    void GetQuaternion(dQuaternion q);
    // return world values
    void GetWorldPosition(dVector3 p);
    void GetWorldQuaternion(dQuaternion q);


    void SetGeomLocation(GeomLocation l) { m_GeomLocation = l; };
    GeomLocation GetGeomLocation() { return m_GeomLocation; };

    void SetContactSoftCFM(dReal cfm) { m_CFM = cfm; };
    dReal GetContactSoftCFM() { return m_CFM; };
    void SetContactSoftERP(dReal erp) { m_ERP = erp; };
    dReal GetContactSoftERP() { return m_ERP; };
    void SetContactMu(dReal mu) { m_Mu = mu; };
    dReal GetContactMu() { return m_Mu; };
    void SetContactBounce(dReal bounce) { m_Bounce = bounce; };
    dReal GetContactBounce() { return m_Bounce; };

    void SetSpringDamp(dReal springConstant, dReal dampingConstant, dReal integrationStep);
    void SetSpringERP(dReal springConstant, dReal ERP, dReal integrationStep);

    void SetAbort(bool abort) { m_Abort = abort; };
    bool GetAbort() { return m_Abort; };

    void AddContact(Contact *contact) { m_ContactList.push_back(contact); }
    std::vector<Contact *> *GetContactList() { return &m_ContactList; }
    void ClearContacts() { m_ContactList.clear(); }

    virtual void Dump();

#ifdef USE_OPENGL
    virtual void Draw() = 0;
#endif

protected:

    dGeomID m_GeomID;

    GeomLocation m_GeomLocation;

    dReal m_CFM;
    dReal m_ERP;
    dReal m_Mu;
    dReal m_Bounce;

    bool m_Abort;

    std::vector<Contact *> m_ContactList;
};


#endif
