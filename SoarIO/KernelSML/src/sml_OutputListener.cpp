/////////////////////////////////////////////////////////////////
// OutputListener class file.
//
// Author: Douglas Pearson, www.threepenny.net
// Date  : September 2004
//
// This class's HandleEvent method is called when
// the agent adds wmes to the output link.
//
/////////////////////////////////////////////////////////////////

#include "sml_OutputListener.h"
#include "sml_Connection.h"
#include "sml_TagWme.h"
#include "IgSKI_WME.h"
#include "IgSKI_Symbol.h"
#include "IgSKI_WMObject.h"

#include <vector>

using namespace sml ;

static char const* GetValueType(egSKISymbolType type)
{
	switch (type)
	{
	case gSKI_DOUBLE: return sml_Names::kTypeDouble ;
	case gSKI_INT:	  return sml_Names::kTypeInt ;
	case gSKI_STRING: return sml_Names::kTypeString ;
	case gSKI_OBJECT: return sml_Names::kTypeID ;
	default: return NULL ;
	}
}

void OutputListener::HandleEvent(egSKIEventId eventId, gSKI::IAgent* agentPtr, egSKIWorkingMemoryChange change, gSKI::tIWmeIterator* wmelist)
{
	if (eventId != gSKIEVENT_OUTPUT_PHASE_CALLBACK)
		return ;

	// Build the SML message we're doing to send.
	ElementXML* pMsg = m_Connection->CreateSMLCommand(sml_Names::kCommand_Output) ;

	// Add the agent parameter and as a side-effect, get a pointer to the <command> tag.  This is an optimization.
	ElementXML_Handle hCommand = m_Connection->AddParameterToSMLCommand(pMsg, sml_Names::kParamAgent, agentPtr->GetName()) ;
	ElementXML command(hCommand) ;

	// We are passed a list of all wmes in the transitive closure (TC) of the output link.
	// We need to decide which of these we've already seen before, so we can just send the
	// changes over to the client (rather than sending the entire TC each time).

	// Reset everything in the current list of tags to "not in use".  After we've processed all wmes,
	// any still in this state have been removed.
	for (OutputTimeTagIter iter = m_TimeTags.begin() ; iter != m_TimeTags.end() ; iter++)
	{
		iter->second = false ;
	}

	// Build the list of WME changes
    for(; wmelist->IsValid(); wmelist->Next())
    {
		// Get the next wme
        gSKI::IWme* pWME = wmelist->GetVal();

		long timeTag = pWME->GetTimeTag() ;

		// See if we've already sent this wme to the client
		OutputTimeTagIter iter = m_TimeTags.find(timeTag) ;

		if (iter != m_TimeTags.end())
		{
			// This is a time tag we've already sent over, so mark it as still being in use
			iter->second = true ;
			continue ;
		}

		// If we reach here we need to send the wme to the client and add it to the list
		// of tags currently in use.
		m_TimeTags[timeTag] = true ;

		// Create the wme tag
		TagWme* pTag = new TagWme() ;

		// Look up the type of value this is
		egSKISymbolType type = pWME->GetValue()->GetType() ;
		char const* pValueType = GetValueType(type) ;

		// For additions we send everything
		pTag->SetIdentifier(pWME->GetOwningObject()->GetId()->GetString()) ;
		pTag->SetAttribute(pWME->GetAttribute()->GetString()) ;
		pTag->SetValue(pWME->GetValue()->GetString(), pValueType) ;
		pTag->SetTimeTag(pWME->GetTimeTag()) ;
		pTag->SetActionAdd() ;

		// Add it as a child of the command tag
		command.AddChild(pTag) ;

		// Values retrieved via "GetVal" have to be released.
		// Ah, but not if they come from an Iterator rather than an IteratorWithRelease.
		// At least, it seems like if I call Release here it causes a crash on exit, while if I don't all seems well.
        //pWME->Release();
	}

	// At this point we check the list of time tags and any which are not marked as "in use" must
	// have been deleted, so we need to send them over to the client as deletions.
	for (OutputTimeTagIter iter = m_TimeTags.begin() ; iter != m_TimeTags.end() ;)
	{
		// Ignore time tags that are still in use.
		if (iter->second == true)
		{
			// We have to do manual iteration because we're deleting elements
			// as we go and that invalidates iterators if we're not careful.
			iter++ ;
			continue ;
		}

		long timeTag = iter->first ;

		// Create the wme tag
		TagWme* pTag = new TagWme() ;

		// For deletions we just send the time tag
		pTag->SetTimeTag(timeTag) ;
		pTag->SetActionRemove() ;

		// Add it as a child of the command tag
		command.AddChild(pTag) ;

		// Delete the entry from the time tag map
		// The returned value points to the next item in the list
		iter = m_TimeTags.erase(iter) ;
	}

	// This is important.  We are working with a subpart of pMsg.
	// If we retain ownership of the handle and delete the object
	// it will release the handle...deleting part of our message.
	command.Detach() ;

#ifdef _DEBUG
	// Generate a text form of the XML so we can look at it in the debugger.
	char* pStr = pMsg->GenerateXMLString(true) ;
	pMsg->DeleteString(pStr) ;
#endif

	// Send the message
	AnalyzeXML response ;
	bool ok = m_Connection->SendMessageGetResponse(&response, pMsg) ;

	// Clean up
	delete pMsg ;

	// Build an output command and send it back to the client
	// I'm not sure what to put in the command.
	// Could build a tree from the top of the o-link.  That seems the most helpful and mark the wmes that have changed with "changed" actions.
	// If so, need to include a depth test, so if change wme at depth 4 nothing is posted.
	// Should think about how the client will handle the message once it comes in.
	// If it's already in tree form within the message the client side will be trivial.  GetNumberChildren().  GetWme(int i).  IsIdentifier().  Repeat.
	// <wme-id id="O3" att="list" value="O6">
	//     <wme id="O6" att="name" value="fred"></wme>
	// </wme-id>

	//	m_Connection->SendAgentCommand(&response, ...) ;


}
