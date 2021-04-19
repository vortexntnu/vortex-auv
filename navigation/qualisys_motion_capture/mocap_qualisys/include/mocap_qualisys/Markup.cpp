#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX

// Markup.cpp: implementation of the NBC_CMarkup class.
//
// NBC_CMarkup Release 6.5 Lite
// Copyright (C) 1999-2003 First Objective Software, Inc. All rights reserved
// This entire notice must be retained in this source code
// Redistributing this source code requires written permission
// This software is provided "as is", with no warranty.
// Latest fixes enhancements and documentation at www.firstobject.com

#include <assert.h>
#include <algorithm>
#include <stdarg.h>
#include <string.h>

#include "Markup.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

void CMarkup::SetIndent(int nIndent)
{
    mnIndent = nIndent;
}

void CMarkup::operator=(const CMarkup& markup)
{
	m_iPosParent = markup.m_iPosParent;
	m_iPos = markup.m_iPos;
	m_iPosChild = markup.m_iPosChild;
	m_iPosFree = markup.m_iPosFree;
	m_nNodeType = markup.m_nNodeType;
    m_aPos.clear();
	m_aPos = markup.m_aPos;
	m_csDoc = markup.m_csDoc;
	MARKUP_SETDEBUGSTATE;
}

bool CMarkup::SetDoc(const char* szDoc)
{
	// Reset indexes
	m_iPosFree = 1;
	ResetPos();

	// Set document text
	if (szDoc)
		m_csDoc = szDoc;
	else
		m_csDoc.erase();

	// Starting size of position array: 1 element per 64 bytes of document
	// Tight fit when parsing small doc, only 0 to 2 reallocs when parsing large doc
	// Start at 8 when creating new document
	std::string::size_type nStartSize = m_csDoc.length() / 64 + 8;
	if (m_aPos.size() < nStartSize)
		m_aPos.resize(nStartSize);

	// Parse document
	bool bWellFormed = false;
	if (m_csDoc.length())
	{
		m_aPos[0].Clear();
		int iPos = x_ParseElem(0);
		if (iPos > 0)
		{
			m_aPos[0].iElemChild = iPos;
			bWellFormed = true;
		}
	}

	// Clear indexes if parse failed or empty document
	if (! bWellFormed)
	{
		m_aPos[0].Clear();
		m_iPosFree = 1;
	}

	ResetPos();

    memset(mtIndent, ' ', sizeof(mtIndent));
    mtIndent[999] = 0;
	return bWellFormed;
}

bool CMarkup::IsWellFormed()
{
	if (!(m_aPos.empty()) && m_aPos[0].iElemChild)
		return true;
	return false;
}

bool CMarkup::FindElem(const char* szName)
{
	// Change current position only if found
	//
	if (!m_aPos.empty())
	{
		int iPos = x_FindElem(m_iPosParent, m_iPos, szName);
		if (iPos)
		{
			// Assign new position
			x_SetPos(m_aPos[iPos].iElemParent, iPos, 0);
			return true;
		}
	}
	return false;
}

bool CMarkup::FindChildElem(const char* szName)
{
	// Change current child position only if found
	//
	// Shorthand: call this with no current main position
	// means find child under root element
	if (! m_iPos)
		FindElem();

	int iPosChild = x_FindElem(m_iPos, m_iPosChild, szName);
	if (iPosChild)
	{
		// Assign new position
		int iPos = m_aPos[iPosChild].iElemParent;
		x_SetPos(m_aPos[iPos].iElemParent, iPos, iPosChild);
		return true;
	}

	return false;
}


std::string CMarkup::GetTagName() const
{
	// Return the tag name at the current main position
	std::string csTagName;


	if (m_iPos)
		csTagName = x_GetTagName(m_iPos);
	return csTagName;
}

bool CMarkup::IntoElem()
{
	// If there is no child position and IntoElem is called it will succeed in release 6.3
	// (A subsequent call to FindElem will find the first element)
	// The following short-hand behavior was never part of EDOM and was misleading
	// It would find a child element if there was no current child element position and go into it
	// It is removed in release 6.3, this change is NOT backwards compatible!
	// if (! m_iPosChild)
	//	FindChildElem();

	if (m_iPos && m_nNodeType == MNT_ELEMENT)
	{
		x_SetPos(m_iPos, m_iPosChild, 0);
		return true;
	}
	return false;
}

bool CMarkup::OutOfElem()
{
	// Go to parent element
	if (m_iPosParent)
	{
		x_SetPos(m_aPos[m_iPosParent].iElemParent, m_iPosParent, m_iPos);
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////
// Private Methods
//////////////////////////////////////////////////////////////////////

int CMarkup::x_GetFreePos()
{
	//
	// This returns the index of the next unused ElemPos in the array
	//
	if (m_iPosFree == (int)m_aPos.size())
		m_aPos.resize(m_iPosFree + m_iPosFree / 2);
	++m_iPosFree;
	return m_iPosFree - 1;
}

int CMarkup::x_ReleasePos()
{
	//
	// This decrements the index of the next unused ElemPos in the array
	// allowing the element index returned by GetFreePos() to be reused
	//
	--m_iPosFree;
	return 0;
}

int CMarkup::x_ParseError(const char* szError, const char* szName)
{
	if (szName)
		m_csError = Format(szError, szName);
	else
		m_csError = szError;
	x_ReleasePos();
	return -1;
}

int CMarkup::x_ParseElem(int iPosParent)
{
	// This is either called by SetDoc, x_AddSubDoc, or itself recursively
	// m_aPos[iPosParent].nEndL is where to start parsing for the child element
	// This returns the new position if a tag is found, otherwise zero
	// In all cases we need to get a new ElemPos, but release it if unused
	//
	int iPos = x_GetFreePos();
	m_aPos[iPos].nStartL = m_aPos[iPosParent].nEndL;
	m_aPos[iPos].iElemParent = iPosParent;
	m_aPos[iPos].iElemChild = 0;
	m_aPos[iPos].iElemNext = 0;

	// Start Tag
	// A loop is used to ignore all remarks tags and special tags
	// i.e. <?xml version="1.0"?>, and <!-- comment here -->
	// So any tag beginning with ? or ! is ignored
	// Loop past ignored tags
	TokenPos token(m_csDoc.c_str());
	token.nNext = m_aPos[iPosParent].nEndL;
	std::string csName;
	while (csName.empty())
	{
		// Look for left angle bracket of start tag
		m_aPos[iPos].nStartL = token.nNext;
		if (! x_FindChar(token.szDoc, m_aPos[iPos].nStartL, '<'))
			return x_ParseError("Element tag not found");

		// Set parent's End tag to start looking from here (or later)
		m_aPos[iPosParent].nEndL = m_aPos[iPos].nStartL;

		// Determine whether this is an element, or bypass other type of node
		token.nNext = m_aPos[iPos].nStartL + 1;
		if (x_FindToken(token))
		{
			if (token.bIsString)
				return x_ParseError("Tag starts with quote");
			char cFirstChar = m_csDoc[token.nL];
			if (cFirstChar == '?' || cFirstChar == '!')
			{
				token.nNext = m_aPos[iPos].nStartL;
				if (! x_ParseNode(token))
					return x_ParseError("Invalid node");
			}
			else if (cFirstChar != '/')
			{
				csName = x_GetToken(token);
				// Look for end of tag
				if (! x_FindChar(token.szDoc, token.nNext, '>'))
					return x_ParseError("End of tag not found");
			}
			else
				return x_ReleasePos(); // probably end tag of parent
		}
		else
			return x_ParseError("Abrupt end within tag");
	}
	m_aPos[iPos].nStartR = token.nNext;

	// Is ending mark within start tag, i.e. empty element?
	if (m_csDoc[m_aPos[iPos].nStartR-1] == '/')
	{
		// Empty element
		// Close tag left is set to ending mark, and right to open tag right
		m_aPos[iPos].nEndL = m_aPos[iPos].nStartR-1;
		m_aPos[iPos].nEndR = m_aPos[iPos].nStartR;
	}
	else // look for end tag
	{
		// Element probably has contents
		// Determine where to start looking for left angle bracket of end tag
		// This is done by recursively parsing the contents of this element
		int iInner, iInnerPrev = 0;
		m_aPos[iPos].nEndL = m_aPos[iPos].nStartR + 1;
		while ((iInner = x_ParseElem(iPos)) > 0)
		{
			// Set links to iInner
			if (iInnerPrev)
				m_aPos[iInnerPrev].iElemNext = iInner;
			else
				m_aPos[iPos].iElemChild = iInner;
			iInnerPrev = iInner;

			// Set offset to reflect child
			m_aPos[iPos].nEndL = m_aPos[iInner].nEndR + 1;
		}
		if (iInner == -1)
			return -1;

		// Look for left angle bracket of end tag
		if (! x_FindChar(token.szDoc, m_aPos[iPos].nEndL, '<'))
			return x_ParseError("End tag of %s element not found", csName.c_str());

		// Look through tokens of end tag
		token.nNext = m_aPos[iPos].nEndL + 1;
		int nTokenCount = 0;
		while (x_FindToken(token))
		{
			++nTokenCount;
			if (! token.bIsString)
			{
				// Is first token not an end slash mark?
				if (nTokenCount == 1 && m_csDoc[token.nL] != '/')
					return x_ParseError("Expecting end tag of element %s", csName.c_str());

				else if (nTokenCount == 2 && ! token.Match(csName.c_str()))
					return x_ParseError("End tag does not correspond to %s", csName.c_str());

				// Else is it a right angle bracket?
				else if (m_csDoc[token.nL] == '>')
					break;
			}
		}

		// Was a right angle bracket not found?
		if (! token.szDoc[token.nL] || nTokenCount < 2)
			return x_ParseError("End tag not completed for element %s", csName.c_str());
		m_aPos[iPos].nEndR = token.nL;
	}

	// Successfully parsed element (and contained elements)
	return iPos;
}

bool CMarkup::x_FindChar(const char* szDoc, int& nChar, char c)
{
	// static function
	const char* pChar = &szDoc[nChar];
	while (*pChar && *pChar != c)
		pChar += 1; //_tclen(pChar);
	nChar = (int)(pChar - szDoc);
	if (! *pChar)
		return false;
	/*
	while (szDoc[nChar] && szDoc[nChar] != c)
		nChar += _tclen(&szDoc[nChar]);
	if (! szDoc[nChar])
		return false;
	*/
	return true;
}

bool CMarkup::x_FindAny(const char* szDoc, int& nChar)
{
	// Starting at nChar, find a non-whitespace char
	// return false if no non-whitespace before end of document, nChar points to end
	// otherwise return true and nChar points to non-whitespace char
	while (szDoc[nChar] && strchr(" \t\n\r", szDoc[nChar]))
		++nChar;
	return szDoc[nChar] != '\0';
}

bool CMarkup::x_FindToken(CMarkup::TokenPos& token)
{
	// Starting at token.nNext, bypass whitespace and find the next token
	// returns true on success, members of token point to token
	// returns false on end of document, members point to end of document
	const char* szDoc = token.szDoc;
	int nChar = token.nNext;
	token.bIsString = false;

	// By-pass leading whitespace
	if (! x_FindAny(szDoc,nChar))
	{
		// No token was found before end of document
		token.nL = nChar;
		token.nR = nChar;
		token.nNext = nChar;
		return false;
	}

	// Is it an opening quote?
	char cFirstChar = szDoc[nChar];
	if (cFirstChar == '\"' || cFirstChar == '\'')
	{
		token.bIsString = true;

		// Move past opening quote
		++nChar;
		token.nL = nChar;

		// Look for closing quote
		x_FindChar(token.szDoc, nChar, cFirstChar);

		// Set right to before closing quote
		token.nR = nChar - 1;

		// Set nChar past closing quote unless at end of document
		if (szDoc[nChar])
			++nChar;
	}
	else
	{
		// Go until special char or whitespace
		token.nL = nChar;
		while (szDoc[nChar] && ! strchr(" \t\n\r<>=\\/?!", szDoc[nChar]))
			nChar += 1; //_tclen(&szDoc[nChar]);

		// Adjust end position if it is one special char
		if (nChar == token.nL)
			++nChar; // it is a special char
		token.nR = nChar - 1;
	}

	// nNext points to one past last char of token
	token.nNext = nChar;
	return true;
}

std::string CMarkup::x_GetToken(const CMarkup::TokenPos& token) const
{
	// The token contains indexes into the document identifying a small substring
	// Build the substring from those indexes and return it
	if (token.nL > token.nR)
		return "";
	return Mid(m_csDoc, token.nL,
		token.nR - token.nL + ((token.nR < (int)(m_csDoc.length())) ? 1 : 0));
}

int CMarkup::x_FindElem(int iPosParent, int iPos, const char* szPath)
{
	// If szPath is NULL or empty, go to next sibling element
	// Otherwise go to next sibling element with matching path
	//
	if (iPos)
		iPos = m_aPos[iPos].iElemNext;
	else
		iPos = m_aPos[iPosParent].iElemChild;

	// Finished here if szPath not specified
	if (szPath == NULL || !szPath[0])
		return iPos;

	// Search
	TokenPos token(m_csDoc.c_str());
	while (iPos)
	{
		// Compare tag name
		token.nNext = m_aPos[iPos].nStartL + 1;
		x_FindToken(token); // Locate tag name
		if (token.Match(szPath))
			return iPos;
		iPos = m_aPos[iPos].iElemNext;
	}
	return 0;
}

int CMarkup::x_ParseNode(CMarkup::TokenPos& token)
{
	// Call this with token.nNext set to the start of the node
	// This returns the node type and token.nNext set to the char after the node
	// If the node is not found or an element, token.nR is not determined
	int nTypeFound = 0;
	const char* szDoc = token.szDoc;
	token.nL = token.nNext;
	if (szDoc[token.nL] == '<')
	{
		// Started with <, could be:
		// <!--...--> comment
		// <!DOCTYPE ...> dtd
		// <?target ...?> processing instruction
		// <![CDATA[...]]> cdata section
		// <NAME ...> element
		//
		if (! szDoc[token.nL+1] || ! szDoc[token.nL+2])
			return 0;
		char cFirstChar = szDoc[token.nL+1];
		const char* szEndOfNode = NULL;
		if (cFirstChar == '?')
		{
			nTypeFound = MNT_PROCESSING_INSTRUCTION;
			szEndOfNode = "?>";
		}
		else if (cFirstChar == '!')
		{
			char cSecondChar = szDoc[token.nL+2];
			if (cSecondChar == '[')
			{
				nTypeFound = MNT_CDATA_SECTION;
				szEndOfNode = "]]>";
			}
			else if (cSecondChar == '-')
			{
				nTypeFound = MNT_COMMENT;
				szEndOfNode = "-->";
			}
			else
			{
				// Document type requires tokenizing because of strings and brackets
				nTypeFound = 0;
				int nBrackets = 0;
				while (x_FindToken(token))
				{
					if (! token.bIsString)
					{
						char cChar = szDoc[token.nL];
						if (cChar == '[')
							++nBrackets;
						else if (cChar == ']')
							--nBrackets;
						else if (nBrackets == 0 && cChar == '>')
						{
							nTypeFound = MNT_DOCUMENT_TYPE;
							break;
						}
					}
				}
				if (! nTypeFound)
					return 0;
			}
		}
		else if (cFirstChar == '/')
		{
			// End tag means no node found within parent element
			return 0;
		}
		else
		{
			nTypeFound = MNT_ELEMENT;
		}

		// Search for end of node if not found yet
		if (szEndOfNode)
		{
			const char* pEnd = strstr(&szDoc[token.nNext], szEndOfNode);
			if (! pEnd)
				return 0; // not well-formed
			token.nNext = (int)(pEnd - szDoc) + (int)strlen(szEndOfNode);
		}
	}
	else if (szDoc[token.nL])
	{
		// It is text or whitespace because it did not start with <
		nTypeFound = MNT_WHITESPACE;
		token.nNext = token.nL;
		if (x_FindAny(szDoc,token.nNext))
		{
			if (szDoc[token.nNext] != '<')
			{
				nTypeFound = MNT_TEXT;
				x_FindChar(szDoc, token.nNext, '<');
			}
		}
	}
	return nTypeFound;
}

std::string CMarkup::x_GetTagName(int iPos) const
{
	// Return the tag name at specified element
	TokenPos token(m_csDoc.c_str());
	token.nNext = m_aPos[iPos].nStartL + 1;
	if (! iPos || ! x_FindToken(token))
		return "";

	// Return substring of document
	return x_GetToken(token);
}

bool CMarkup::x_FindAttrib(CMarkup::TokenPos& token, const char* szAttrib) const
{
	// If szAttrib is NULL find next attrib, otherwise find named attrib
	// Return true if found
	int nAttrib = 0;
	for (int nCount = 0; x_FindToken(token); ++nCount)
	{
		if (! token.bIsString)
		{
			// Is it the right angle bracket?
			char cChar = m_csDoc[token.nL];
			if (cChar == '>' || cChar == '/' || cChar == '?')
				break; // attrib not found

			// Equal sign
			if (cChar == '=')
				continue;

			// Potential attribute
			if (! nAttrib && nCount)
			{
				// Attribute name search?
				if (! szAttrib || ! szAttrib[0])
					return true; // return with token at attrib name

				// Compare szAttrib
				if (token.Match(szAttrib))
					nAttrib = nCount;
			}
		}
		else if (nAttrib && nCount == nAttrib + 2)
		{
			return true;
		}
	}

	// Not found
	return false;
}

std::string CMarkup::x_GetAttrib(int iPos, const char* szAttrib) const
{
	// Return the value of the attrib
	TokenPos token(m_csDoc.c_str());
	if (iPos && m_nNodeType == MNT_ELEMENT)
		token.nNext = m_aPos[iPos].nStartL + 1;
	else
		return "";

	if (szAttrib && x_FindAttrib(token, szAttrib))
		return x_TextFromDoc(token.nL, token.nR - ((token.nR < (int)(m_csDoc.length())) ? 0 : 1));
	return "";
}

bool CMarkup::x_SetAttrib(int iPos, const char* szAttrib, const char* szValue)
{
	// Set attribute in iPos element
	TokenPos token(m_csDoc.c_str());
	int nInsertAt;
	if (iPos && m_nNodeType == MNT_ELEMENT)
	{
		token.nNext = m_aPos[iPos].nStartL + 1;
		nInsertAt = m_aPos[iPos].nStartR - (m_aPos[iPos].IsEmptyElement()?1:0);
	}
	else
		return false;

	// Create insertion text depending on whether attribute already exists
	int nReplace = 0;
	std::string csInsert;
	if (x_FindAttrib(token, szAttrib))
	{
		// Replace value only
		// Decision: for empty value leaving attrib="" instead of removing attrib
		csInsert = x_TextToDoc(szValue, true);
		nInsertAt = token.nL;
		nReplace = token.nR-token.nL+1;
	}
	else
	{
		// Insert string name value pair
		std::string csFormat;
		csFormat = " ";
		csFormat += szAttrib;
		csFormat += "=\"";
		csFormat += x_TextToDoc(szValue, true);
		csFormat += "\"";
		csInsert = csFormat;
	}

	x_DocChange(nInsertAt, nReplace, csInsert);
	int nAdjust = (int)csInsert.length() - nReplace;
	m_aPos[iPos].nStartR += nAdjust;
	m_aPos[iPos].AdjustEnd(nAdjust);
	x_Adjust(iPos, nAdjust);
	MARKUP_SETDEBUGSTATE;
	return true;
}

std::string CMarkup::x_GetData(int iPos) const
{

	// Return a string representing data between start and end tag
	// Return empty string if there are any children elements
	if (! m_aPos[iPos].iElemChild && ! m_aPos[iPos].IsEmptyElement())
	{
		// See if it is a CDATA section
		const char* szDoc = (const char*)(m_csDoc.c_str());
		int nChar = m_aPos[iPos].nStartR + 1;
		if (x_FindAny(szDoc, nChar) && szDoc[nChar] == '<'
				&& nChar + 11 < m_aPos[iPos].nEndL
				&& strncmp(&szDoc[nChar], "<![CDATA[", 9) == 0)
		{
			nChar += 9;
			int nEndCDATA = (int)m_csDoc.find("]]>", nChar);
			if (nEndCDATA != -1 && nEndCDATA < m_aPos[iPos].nEndL)
			{
				return Mid(m_csDoc, nChar, nEndCDATA - nChar);
			}
		}
		return x_TextFromDoc(m_aPos[iPos].nStartR+1, m_aPos[iPos].nEndL-1);
	}
	return "";
}

std::string CMarkup::x_TextToDoc(const char* szText, bool bAttrib) const
{
	// Convert text as seen outside XML document to XML friendly
	// replacing special characters with ampersand escape codes
	// E.g. convert "6>7" to "6&gt;7"
	//
	// &lt;   less than
	// &amp;  ampersand
	// &gt;   greater than
	//
	// and for attributes:
	//
	// &apos; apostrophe or single quote
	// &quot; double quote
	//
	static const char* szaReplace[] = { "&lt;", "&amp;", "&gt;", "&apos;", "&quot;" };
	const char* pFind = bAttrib ? "<&>\'\"" : "<&>";
	std::string csText;
	const char* pSource = szText;
	int nDestSize = (int)strlen(pSource);
	nDestSize += nDestSize / 10 + 7;
	char* pDest = GetBuffer(csText, nDestSize);
	int nLen = 0;
	char cSource = *pSource;
	const char* pFound;
	while (cSource)
	{
		if (nLen > nDestSize - 6)
		{
			ReleaseBuffer(csText, nLen);
			nDestSize *= 2;
			pDest = GetBuffer(csText, nDestSize);
		}
		if ((pFound = strchr(pFind,cSource)) != NULL)
		{
			pFound = szaReplace[pFound-pFind];
#ifdef _WIN32
			strcpy_s(&pDest[nLen], nDestSize, pFound);
#else
			strncpy(&pDest[nLen], pFound, nDestSize);
#endif
			nLen += (int)strlen(pFound);
		}
		else
		{
			pDest[nLen] = *pSource;
			nLen += 1; //_tclen(pSource);
		}
		pSource += 1; //_tclen(pSource);
		cSource = *pSource;
	}
	ReleaseBuffer(csText, nLen);
	return csText;
}

std::string CMarkup::x_TextFromDoc(int nLeft, int nRight) const
{
	// Convert XML friendly text to text as seen outside XML document
	// ampersand escape codes replaced with special characters e.g. convert "6&gt;7" to "6>7"
	// Conveniently the result is always the same or shorter in byte length
	//
	static const char* szaCode[] = { "lt;", "amp;", "gt;", "apos;", "quot;" };
	static int anCodeLen[] = { 3,4,3,5,5 };
	static const char* szSymbol  = "<&>\'\"";
	std::string csText;
	const char* pSource = m_csDoc.c_str();
	int nDestSize = nRight - nLeft + 1;
	char* pDest = GetBuffer(csText, nDestSize);
	int nLen = 0;
	int nCharLen;
	int nChar = nLeft;
	while (nChar <= nRight)
	{
		if (pSource[nChar] == '&')
		{
			// Look for matching &code;
			bool bCodeConverted = false;
			for (int nMatch = 0; nMatch < 5; ++nMatch)
			{
				if (nChar <= nRight - anCodeLen[nMatch]
					&& strncmp(szaCode[nMatch],&pSource[nChar+1],anCodeLen[nMatch]) == 0)
				{
					// Insert symbol and increment index past ampersand semi-colon
					pDest[nLen++] = szSymbol[nMatch];
					nChar += anCodeLen[nMatch] + 1;
					bCodeConverted = true;
					break;
				}
			}

			// If the code is not converted, leave it as is
			if (! bCodeConverted)
			{
				pDest[nLen++] = '&';
				++nChar;
			}
		}
		else // not &
		{
			nCharLen = 1; //_tclen(&pSource[nChar]);
			pDest[nLen] = pSource[nChar];
			nLen += nCharLen;
			nChar += nCharLen;
		}
	}
	ReleaseBuffer(csText, nLen);
	return csText;
}

void CMarkup::x_DocChange(int nLeft, int nReplace, const std::string& csInsert)
{
	// Insert csInsert int m_csDoc at nLeft replacing nReplace chars
	// Do this with only one buffer reallocation if it grows
	//
	int nDocLength = (int)m_csDoc.length();
	int nInsLength = (int)csInsert.length();

	// Make sure nLeft and nReplace are within bounds
	nLeft = std::max(0, std::min(nLeft, nDocLength));
	nReplace = std::max(0, std::min(nReplace, nDocLength-nLeft));

	// Get pointer to buffer with enough room
	int nNewLength = nInsLength + nDocLength - nReplace;
	int nBufferLen = nNewLength;
	char* pDoc = GetBuffer(m_csDoc, nBufferLen);

	// Move part of old doc that goes after insert
	if (nLeft+nReplace < nDocLength)
		memmove(&pDoc[nLeft+nInsLength], &pDoc[nLeft+nReplace], (nDocLength-nLeft-nReplace)*sizeof(char));

	// Copy insert
	memcpy(&pDoc[nLeft], csInsert.c_str(), nInsLength*sizeof(char));

	// Release
	ReleaseBuffer(m_csDoc, nNewLength);
}

void CMarkup::x_Adjust(int iPos, int nShift, bool bAfterPos)
{
	// Loop through affected elements and adjust indexes
	// Algorithm:
	// 1. update children unless bAfterPos
	//    (if no children or bAfterPos is true, end tag of iPos not affected)
	// 2. update next siblings and their children
	// 3. go up until there is a next sibling of a parent and update end tags
	// 4. step 2
	int iPosTop = m_aPos[iPos].iElemParent;
	bool bPosFirst = bAfterPos; // mark as first to skip its children
	while (iPos)
	{
		// Were we at containing parent of affected position?
		bool bPosTop = false;
		if (iPos == iPosTop)
		{
			// Move iPosTop up one towards root
			iPosTop = m_aPos[iPos].iElemParent;
			bPosTop = true;
		}

		// Traverse to the next update position
		if (! bPosTop && ! bPosFirst && m_aPos[iPos].iElemChild)
		{
			// Depth first
			iPos = m_aPos[iPos].iElemChild;
		}
		else if (m_aPos[iPos].iElemNext)
		{
			iPos = m_aPos[iPos].iElemNext;
		}
		else
		{
			// Look for next sibling of a parent of iPos
			// When going back up, parents have already been done except iPosTop
			while ((iPos=m_aPos[iPos].iElemParent) != 0 && iPos != iPosTop)
				if (m_aPos[iPos].iElemNext)
				{
					iPos = m_aPos[iPos].iElemNext;
					break;
				}
		}
		bPosFirst = false;

		// Shift indexes at iPos
		if (iPos != iPosTop)
			m_aPos[iPos].AdjustStart(nShift);
		m_aPos[iPos].AdjustEnd(nShift);
	}
}

void CMarkup::x_LocateNew(int iPosParent, int& iPosRel, int& nOffset, int nLength, int nFlags)
{
	// Determine where to insert new element or node
	//
	bool bInsert = (nFlags&1)?true:false;
	bool bHonorWhitespace = (nFlags&2)?true:false;

	std::string::size_type nStartL;
	if (nLength)
	{
		// Located at a non-element node
		if (bInsert)
			nStartL = nOffset;
		else
			nStartL = nOffset + nLength;
	}
	else if (iPosRel)
	{
		// Located at an element
		if (bInsert) // precede iPosRel
			nStartL = m_aPos[iPosRel].nStartL;
		else // follow iPosRel
			nStartL = m_aPos[iPosRel].nEndR + 1;
	}
	else if (! iPosParent)
	{
		// Outside of all elements
		if (bInsert)
			nStartL = 0;
		else
			nStartL = m_csDoc.length();
	}
	else if (m_aPos[iPosParent].IsEmptyElement())
	{
		// Parent has no separate end tag, so split empty element
		nStartL = m_aPos[iPosParent].nStartR;
	}
	else
	{
		if (bInsert) // after start tag
			nStartL = m_aPos[iPosParent].nStartR + 1;
		else // before end tag
			nStartL = m_aPos[iPosParent].nEndL;
	}

	// Go up to start of next node, unless its splitting an empty element
	if (! bHonorWhitespace && ! m_aPos[iPosParent].IsEmptyElement())
	{
		const char* szDoc = (const char*)m_csDoc.c_str();
		int nChar = (int)nStartL;
		if (! x_FindAny(szDoc,nChar) || szDoc[nChar] == '<')
			nStartL = nChar;
	}

	// Determine iPosBefore
	int iPosBefore = 0;
	if (iPosRel)
	{
		if (bInsert)
		{
			// Is iPosRel past first sibling?
			int iPosPrev = m_aPos[iPosParent].iElemChild;
			if (iPosPrev != iPosRel)
			{
				// Find previous sibling of iPosRel
				while (m_aPos[iPosPrev].iElemNext != iPosRel)
					iPosPrev = m_aPos[iPosPrev].iElemNext;
				iPosBefore = iPosPrev;
			}
		}
		else
		{
			iPosBefore = iPosRel;
		}
	}
	else if (m_aPos[iPosParent].iElemChild)
	{
		if (! bInsert)
		{
			// Find last element under iPosParent
			int iPosLast = m_aPos[iPosParent].iElemChild;
			int iPosNext = iPosLast;
			while (iPosNext)
			{
				iPosLast = iPosNext;
				iPosNext = m_aPos[iPosNext].iElemNext;
			}
			iPosBefore = iPosLast;
		}
	}

	nOffset = (int)nStartL;
	iPosRel = iPosBefore;
}

bool CMarkup::x_AddElem(const char* szName, const char* szValue, bool bInsert, bool bAddChild)
{
	if (bAddChild)
	{
		// Adding a child element under main position
		if (! m_iPos)
			return false;
	}
	else if (m_iPosParent == 0)
	{
		// Adding root element
		if (IsWellFormed())
			return false;


		// Locate after any version and DTD
		m_aPos[0].nEndL = (int)m_csDoc.length();
	}

	// Locate where to add element relative to current node
	int iPosParent, iPosBefore, nOffset = 0, nLength = 0;
	if (bAddChild)
	{
		iPosParent = m_iPos;
		iPosBefore = m_iPosChild;
	}
	else
	{
		iPosParent = m_iPosParent;
		iPosBefore = m_iPos;
	}
	int nFlags = bInsert?1:0;
	x_LocateNew(iPosParent, iPosBefore, nOffset, nLength, nFlags);
    // LocateNew: in case of an empty parent it finds the end of the start tag (sort of)
    // in case of a non-empty parent it finds the char before the start of the end tag.


    // Find out the indent we need:
    int nTopParent = iPosParent;
    int nLevel = 0;
    while (nTopParent)
    {
        nTopParent = m_aPos[nTopParent].iElemParent;
        nLevel++;
    }
    int nIndentChars = nLevel * mnIndent;
    mtIndent[ nIndentChars ] = 0;


    bool bEmptyParent = m_aPos[iPosParent].IsEmptyElement();
	if (bEmptyParent || m_aPos[iPosParent].nStartR + 1 == m_aPos[iPosParent].nEndL)
    {
    	nOffset += 2;
    }
    else
    {
        if ((nOffset < (int)(m_csDoc.length())) && (0 < nOffset) && 
            (' ' == m_csDoc[nOffset-1]))
        {
            while ((0 < nOffset) && (' ' == m_csDoc[nOffset-1]))
                --nOffset;
        }
    }

    // Create element and modify positions of affected elements
    // If no szValue is specified, an empty element is created
    // i.e. either <NAME>value</NAME> or <NAME/>
    int iPos = x_GetFreePos();
    m_aPos[iPos].nStartL = nOffset + nIndentChars;
	
	// Set links
	m_aPos[iPos].iElemParent = iPosParent;
	m_aPos[iPos].iElemChild = 0;
	m_aPos[iPos].iElemNext = 0;
	if (iPosBefore)
	{
		// Link in after iPosBefore
		m_aPos[iPos].iElemNext = m_aPos[iPosBefore].iElemNext;
		m_aPos[iPosBefore].iElemNext = iPos;
	}
	else
	{
		// First child
		m_aPos[iPos].iElemNext = m_aPos[iPosParent].iElemChild;
		m_aPos[iPosParent].iElemChild = iPos;
	}

	// Create string for insert
	std::string csInsert;
	int nLenName = (int)strlen(szName);
	int nLenValue = szValue ? (int)strlen(szValue) : 0;
	if (! nLenValue)
	{
		// <NAME/> empty element
		csInsert = mtIndent; 
        csInsert += "<";
		csInsert += szName;
		csInsert += "/>\r\n";
		m_aPos[iPos].nStartR = m_aPos[iPos].nStartL + nLenName + 2;
		m_aPos[iPos].nEndL = m_aPos[iPos].nStartR - 1;
		m_aPos[iPos].nEndR = m_aPos[iPos].nEndL + 1;
	}
	else
	{
		// <NAME>value</NAME>
		std::string csValue = x_TextToDoc(szValue);
		nLenValue = (int)csValue.length();
        csInsert = mtIndent;
        csInsert += "<";
		csInsert += szName;
		csInsert += ">";
		csInsert += csValue;
		csInsert += "</";
		csInsert += szName;
		csInsert += ">\r\n";
		m_aPos[iPos].nStartR = m_aPos[iPos].nStartL + nLenName + 1;
		m_aPos[iPos].nEndL = m_aPos[iPos].nStartR + nLenValue + 1;
		m_aPos[iPos].nEndR = m_aPos[iPos].nEndL + nLenName + 2;
	}
    mtIndent[ nIndentChars ] = ' ';


    

	// Insert
	int nReplace = 0, nLeft = m_aPos[iPos].nStartL;
	if (bEmptyParent)
	{
		std::string csParentTagName = x_GetTagName(iPosParent);
		std::string csFormat;
		csFormat = ">\r\n";
		csFormat += csInsert;
        mtIndent[ nIndentChars - mnIndent ] = 0;
        csFormat += mtIndent;
        mtIndent[ nIndentChars - mnIndent ] = ' ';
		csFormat += "</";
		csFormat += csParentTagName;
		csInsert = csFormat;
		nLeft = m_aPos[iPosParent].nStartR - 1;
		nReplace = 1;
		// x_Adjust is going to update all affected indexes by one amount
		// This will satisfy all except the empty parent
		// Here we pre-adjust for the empty parent
		// The empty tag slash is removed
		m_aPos[iPosParent].nStartR -= 1;
		// For the newly created end tag, see the following example:
		// <A/> (len 4) becomes <A><B/></A> (len 11)
		// In x_Adjust everything will be adjusted 11 - 4 = 7
		// But the nEndL of element A should only be adjusted 5
		m_aPos[iPosParent].nEndL -= (int)(csParentTagName.length() + 1);
	}
	else if (m_aPos[iPosParent].nStartR + 1 == m_aPos[iPosParent].nEndL)
	{
        // Empty parent, but with an end tag following right after.
        csInsert = "\r\n" + csInsert;
        mtIndent[ nIndentChars - mnIndent ] = 0;
        csInsert += mtIndent;
        mtIndent[ nIndentChars - mnIndent ] = ' ';
		nLeft = m_aPos[iPosParent].nStartR + 1;
	}
    else
    {
        nLeft -= nIndentChars;
    }
    
	x_DocChange(nLeft, nReplace, csInsert);
	x_Adjust(iPos, (int)csInsert.length() - nReplace);

	if (bAddChild)
		x_SetPos(m_iPosParent, iPosParent, iPos);
	else
		x_SetPos(iPosParent, iPos, 0);
	return true;
}

#ifndef _WIN32
int _vscprintf(const char* format, va_list pargs)
{
    int retval;
    va_list argcopy;
    va_copy(argcopy, pargs);
    retval = vsnprintf(NULL, 0, format, argcopy);
    va_end(argcopy);
    return retval;
}
#endif

std::string CMarkup::Format(const char *fmt, ...)
{
    std::string retStr;
    if (NULL != fmt)
    {
        va_list marker;

        // initialize variable arguments
        va_start(marker, fmt);

        // Get formatted string length adding one for NULL
        int len = _vscprintf(fmt, marker) + 1;

        // Create a char vector to hold the formatted string.
        std::vector<char> buffer(len, '\0');
#ifdef _WIN32
        int nWritten = _vsnprintf_s(&buffer[0], buffer.size(), len, fmt, marker);
#else
        int nWritten = vsnprintf(&buffer[0], len, fmt, marker);
#endif
        if (nWritten > 0)
        {
            retStr = &buffer[0];
        }

        // Reset variable arguments
        va_end(marker);
    }
    return retStr;
}

std::string CMarkup::Mid(const std::string &tStr, int nFirst) const
{
    return Mid(tStr, nFirst, (int)tStr.length() - nFirst);
}

std::string CMarkup::Mid(const std::string &tStr, int nFirst, int nCount) const
{
    if (nFirst < 0)
    {
        nFirst = 0;
    }
    if (nCount < 0)
    {
        nCount = 0;
    }

    int nSize = static_cast<int>(tStr.size());

    if (nFirst + nCount > nSize)
    {
        nCount = nSize - nFirst;
    }

    if (nFirst > nSize)
    {
        std::string tStrEmpty;
        return tStrEmpty;
    }

    assert(nFirst >= 0);
    assert(nFirst + nCount <= nSize);

    return tStr.substr(nFirst, nCount);
}

char* CMarkup::GetBuffer(std::string &tStr, int nMinLen) const
{
    if (static_cast<int>(tStr.size()) < nMinLen)
    {
        tStr.resize(nMinLen);
    }

    return const_cast<char*>(tStr.c_str()); //tStr.empty() ? const_cast<char*>(tStr.c_str()) : &(tStr.at(0));
}

void CMarkup::ReleaseBuffer(std::string &tStr, int nNewLen) const
{
    tStr.resize(nNewLen > -1 ? nNewLen : strlen(tStr.c_str()));
}

bool CMarkup::TokenPos::Match(const char* szName) const
{
	int nLen = nR - nL + 1;
	return ((strncmp(&szDoc[nL], szName, nLen) == 0)
		&& (szName[nLen] == '\0' || strchr(" =/[", szName[nLen])));
}
