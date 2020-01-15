// Simple Ringbuffer
// Copyright (C) 2015 Florian Jung
// All rights reserved. Licensed under the 3-clause-BSD-license as follows:
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software without
//    specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include <math.h>

class Ringbuffer
{
	public:

	Ringbuffer(int size)
	{
		this->size = size;
		idx = 0;
		buf = new double[size];
		for (int i=0; i<size; i++)
			buf[i] = 0;
		sum_ = 0.0;
		sum_valid = false;
	}

	~Ringbuffer()
	{
		delete [] buf;
	}

	double sum()
	{
		if (!sum_valid)
		{
			sum_=0.0;
			for (int i=0; i<size; i++)
				sum_ += buf[i];
			sum_valid=true;
		}

		return sum_;
	}
	
	double get()
	{
		return sum()/size;
	}
	
	double front()
	{
		return buf[idx];
	}

	void put(double val)
	{
		buf[idx] = val;
		idx = (idx+1) % size;
		sum_valid = false;
	}

	void set(double val)
	{
		for (int i=0; i<size; i++)
			buf[i]=val;
		sum_ = size*val;
		sum_valid=true;
	}

	void add(double val)
	{
		for (int i=0; i<size; i++)
			buf[i]+=val;
		sum_ += size*val;
	}

	private:
		double* buf;
		int idx;
		int size;
		double sum_;
		bool sum_valid;
};

class ModuloRingbuffer
{
	private:
		Ringbuffer* rb;
		double low,upp,span;

	public:
	
	ModuloRingbuffer(int size, double low, double upp)
	{
		rb = new Ringbuffer(size);
		this->low=low;
		this->upp=upp;
		this->span=upp-low;
		rb->set(_fixup_range(0.0));
	}

	~ModuloRingbuffer()
	{
		delete rb;
	}

	double _fixup_range(double val)
	{
		while (val < low) val+=span;
		while (val>= upp) val-=span;
		return val;
	}

	double get()
	{
		return rb->get();
	}

	void put(double val)
	{
		val=_fixup_range(val);

		// direct way
		double dist1 = val - get();
		// over borders
		double dist2 = dist1 + span;
		double dist3 = dist1 - span;

		if (fabs(dist1) <= fabs(dist2) && fabs(dist1) <= fabs(dist3))
			rb->put(val);
		else if (fabs(dist2) <= fabs(dist3))
			rb->put(val+span);
		else
			rb->put(val-span);

		while (get() < low) rb->add(span);
		while (get()>= upp) rb->add(-span);
	}

	void set(double val)
	{
		rb->set(_fixup_range(val));
	}

	void add(double val)
	{
		rb->add(val);
		
		while (get() < low) rb->add(span);
		while (get()>= upp) rb->add(-span);
	}
};
