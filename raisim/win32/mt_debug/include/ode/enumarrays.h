/*************************************************************************
 *                                                                       *
 * ODER's Utilities Library. Copyright (C) 2008 Oleh Derevenko.          *
 * All rights reserved.  e-mail: odar@eleks.com (change all "a" to "e")  *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 3 of the License, or (at    *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE-LESSER.TXT. Since LGPL is the extension of GPL     *
 *       the text of GNU General Public License is also provided for     *
 *       your information in file LICENSE.TXT.                           *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *   (3) The zlib/libpng license that is included with this library in   *
 *       the file LICENSE-ZLIB.TXT                                       *
 *                                                                       *
 * This library is distributed WITHOUT ANY WARRANTY, including implied   *
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.      *
 * See the files LICENSE.TXT and LICENSE-LESSER.TXT or LICENSE-BSD.TXT   *
 * or LICENSE-ZLIB.TXT for more details.                                 *
 *                                                                       *
 *************************************************************************/

#ifndef __OU_ENUMARRAYS_H_INCLUDED
#define __OU_ENUMARRAYS_H_INCLUDED

//////////////////////////////////////////////////////////////////////////
// Helper template definitions
	
template<typename ElementType>
struct CTypeStandardEqual
{
	bool  operator ()(const ElementType &etLeftElement, const ElementType &etRightElement) const
	{
		return etLeftElement == etRightElement;
	}
};

template<typename ElementType>
struct CTypeStandardLess
{
	bool  operator ()(const ElementType &etLeftElement, const ElementType &etRightElement) const
	{
		return etLeftElement < etRightElement;
	}
};


//////////////////////////////////////////////////////////////////////////
// CEnumUnsortedElementArray definition

/*
 *	Implementation Note:
 *	The array is intended to store static constant data. 
 *	Therefore CElementEqualType should not ever need a nontrivial constructor
 *	and it is acceptable to have it as template parameter.
 */

template<typename EnumType, const EnumType EnumMax, typename ElementType, const int Instance=0, class CElementEqualType=CTypeStandardEqual<ElementType> >
class CEnumUnsortedElementArray
{
public:
	 CEnumUnsortedElementArray()
	{
	}
	
public:
	static inline const EnumType  
	/*const EnumType */Decode(const ElementType &etValue)
	{
		const ElementType *itElementFound = FindValueSequentially(m_aetElementArray, m_aetElementArray + EnumMax, etValue);

		EnumType etResult = (EnumType)(itElementFound - m_aetElementArray);
		return etResult;
	}
	
	static inline const ElementType & 
	/*const ElementType &*/Encode(const EnumType &etValue)
	{
//		dIASSERT(sizeof(EnumType) <= sizeof(int));
//		dIASSERT(OU_IN_INT_RANGE(etValue, 0, EnumMax));

		return m_aetElementArray[etValue];
	}
	
	static inline bool  
	/*bool */IsValidDecode(const EnumType &etValue)
	{
		return etValue != EnumMax;
	}
	
	static inline const ElementType * 
	/*const ElementType **/GetStoragePointer()
	{
		return m_aetElementArray;
	}
	
private:
	static const ElementType * FindValueSequentially(const ElementType *petArrayBegin, const ElementType *petArrayEnd, const ElementType &etValue)
	{
		const CElementEqualType etElementEqual = CElementEqualType();

		const ElementType *petCurrentElement = petArrayBegin;

		for (; petCurrentElement != petArrayEnd; ++petCurrentElement)
		{
			if (etElementEqual(*petCurrentElement, etValue))
			{
				break;
			}
		}

		return petCurrentElement;
	}

private:
	static const ElementType m_aetElementArray[];
};
	

//////////////////////////////////////////////////////////////////////////
// CEnumSortedElementArray definition

/*
 *	Implementation Note:
 *	The array is intended to store static constant data. 
 *	Therefore CElementLessType and CElementEqualType should not ever need 
 *	a nontrivial constructor and it is acceptable to have them 
 *	as template parameters.
 */

template<typename EnumType, const EnumType EnumMax, typename ElementType, const int Instance=0, class CElementLessType=CTypeStandardLess<ElementType> >
class CEnumSortedElementArray
{
public:
	 CEnumSortedElementArray()
	{
	}
	
	static inline const EnumType 
	/*const EnumType */Decode(const ElementType &etValue)
	{
		const CElementLessType ltElementLess = CElementLessType();
		
		EnumType etResult = EnumMax;

		const ElementType *itElementFound = FindValueLowerBound(m_aetElementArray, m_aetElementArray + EnumMax, etValue);
		
		if (itElementFound != m_aetElementArray + EnumMax)
		{
			if (!ltElementLess(etValue, *itElementFound))
			{
				etResult = (EnumType)(itElementFound - m_aetElementArray);
			}
		}
		
		return etResult;
	}
	
	static inline const ElementType & 
	/*const ElementType &*/Encode(const EnumType &etValue)
	{
//		dIASSERT(sizeof(EnumType) <= sizeof(int));
//		dIASSERT(OU_IN_INT_RANGE(etValue, 0, EnumMax));

		return m_aetElementArray[etValue];
	}
	
	static inline bool  
	/*bool */IsValidDecode(const EnumType &etValue)
	{
		return etValue != EnumMax;
	}
	
	static inline const ElementType * 
	/*const ElementType **/GetStoragePointer()
	{
		return m_aetElementArray;
	}

private:
	static const ElementType * FindValueLowerBound(const ElementType *petArrayBegin, const ElementType *petArrayEnd, const ElementType &etValue)
	{
		const CElementLessType ltElementLess = CElementLessType();

		const ElementType *petCurrentRangeBegin = petArrayBegin;
		const ElementType *petCurrentRangeEnd = petArrayEnd;

		while (petCurrentRangeBegin != petCurrentRangeEnd)
		{
			const ElementType *petCurrentRangeMiddle = petCurrentRangeBegin + (petCurrentRangeEnd - petCurrentRangeBegin) / 2;

			if (ltElementLess(*petCurrentRangeMiddle, etValue))
			{
				petCurrentRangeBegin = petCurrentRangeMiddle + 1;
			}
			else
			{
				petCurrentRangeEnd = petCurrentRangeMiddle;
			}
		}

		return petCurrentRangeBegin;
	}

private:
	static const ElementType m_aetElementArray[];
};

#endif // #ifndef __OU_ENUMARRAYS_H_INCLUDED
