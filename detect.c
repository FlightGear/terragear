#include "plugin.h"
// Triangle-stripdetection made simple. By Joost Bloemen, Copyright Vimana BV
// If you can improve the algorithm, be sure to let me know you can reach me
// at research@3dtop.com

struct BMF_SURFACE{
    USHORT p0, p1, p2;
};

struct Hit{
    USHORT hits;
    USHORT t[3];
    UCHAR mask[3];
    BMF_SURFACE p[3];
    BOOL Stripped;
}* HitList;

// INPUT:
//ilist is the list with indices of the original triangles, will be broken down to
//	  only the triangles that can 't be stripped, at the end.
//length is the number of triangles in ilist.
// OUTPUT:
//nr-strips is the number of strips we detected, they have a minimum of two triangles
//strips_length contains an array[nr_strips] of strip-lengths, strip1, strip2 strip3 etc...
//nr_indices is the total number of vertice-indices we have in the array "stripindex":
//stripindex is a long list of strip1-indices, strip2-indices, strip3-indices etc...
//return-value is the new number of seperate triangles in ilist.
USHORT DetectStrips(BMF_SURFACE * ilist, USHORT length,
		    USHORT * nr_strips, USHORT * strip_length,
		    USHORT * nr_indices, USHORT * stripindex)
{
    USHORT lastp0,lastp1,lastp2;
    USHORT n,nr,trynr;
    USHORT p0,p1,p2;
    USHORT i,k,next,p;
    UCHAR mask;
    BMF_SURFACE * temp, * tempEnd;
    struct Hit * tempHitList;
    // First part makes a hitlist per triangle, how many sides(hits) a triangle has shared,
    // the indexes of the hits, up to 3 masks that define wich points the triangles each share
    // and the points of each shared triangle.
    HitList=(struct Hit *)GlobalAlloc(GMEM_FIXED|GMEM_ZEROINIT,length*sizeof(struct Hit));
    tempHitList=HitList;
    for(n=0;n<length;n++,tempHitList++)
	{
	    p0=ilist[n].p0;
	    p1=ilist[n].p1;
	    p2=ilist[n].p2;
	    temp=ilist;
	    tempEnd=ilist+length;
	    while(temp<tempEnd)
		{USHORT p;
		// The following piece of code accounts for 99% of the time when converting large models
		nr=2;                               // count-down, we only want the ones with two points the same
		mask=0;
		if ((p=temp->p0)==p0){mask|=0x11;nr--;}
		else {if (p==p1){mask|=0x12;nr--;}
		else {if (p==p2){mask|=0x14;nr--;}}
		}
		if ((p=temp->p1)==p0){mask|=0x21;nr--;}
		else {if (p==p1){mask|=0x22;nr--;}
		else {if (p==p2){mask|=0x24;nr--;}}
		}
		if ((p=temp->p2)==p0){mask|=0x41;nr--;}
		else {if (p==p1){mask|=0x42;nr--;}
		else {if (p==p2){mask|=0x44;nr--;}}
		}

		// That's it
		if(!nr)																	//this doesn't happen very often
		    {USHORT nrs=tempHitList->hits;
		    //Rotate and save points so that p0 and p1 point to points that are the same.
		    // to check later, if we can make a strip of this
		    switch (mask>>4){
		    case 5:
			tempHitList->p[nrs].p1=temp->p0;
			tempHitList->p[nrs].p0=temp->p2;
			tempHitList->p[nrs].p2=temp->p1;
			break;

		    case 6:
			tempHitList->p[nrs].p0=temp->p1;
			tempHitList->p[nrs].p1=temp->p2;
			tempHitList->p[nrs].p2=temp->p0;
			break;

		    case 3:
			tempHitList->p[nrs].p0=temp->p0;
			tempHitList->p[nrs].p1=temp->p1;
			tempHitList->p[nrs].p2=temp->p2;
			break;
		    }
		    tempHitList->t[nrs]=temp-ilist;         // (temp-ilist) is actually the triangle-number 
		    tempHitList->mask[nrs]=mask;            // mask is for later
		    if(++tempHitList->hits==3)temp=tempEnd;		// break while-loop if we have 3 hits
		    }
		temp++;
		}
	}
    // Next:
    // Start with trying to make a strip of all triangles with 1 hit as a starting point,
    // then 2 , finally 3.
    // That's all.
    for(trynr=1;trynr<=3;trynr++)
	{
	    for(p=0;p<length;p++)
		{
		    if(HitList[p].hits==trynr&&!HitList[p].Stripped)
			{
			    n=p;                              		// n is first triangle of possible strip
			    i=0;                                  // i is triangle-counter of this possible strip
			    k=10;                                 // found a matching triangle
			    while(k>=10)                          // while found a triangle
				{for (k=0;k<trynr;k++)    					// try all possible triangles
				    {next=HitList[n].t[k];						// possible triangle
				    if(!HitList[next].Stripped)				// not included yet ?
					{if(!i)							// if testing with first triangle, it must be rotated so that
					    // points that are the same as next triangle are p1 and p2
					    {switch (HitList[n].mask[k]&0x0f){
					    case 6:
						lastp0=ilist[p].p0;         // lastp0-p2 is first triangle of strip
						lastp1=ilist[p].p1;         // is a local triangle that defines the last
						lastp2=ilist[p].p2;					// triangle added to the strip
						break;

					    case 3:
						lastp1=ilist[p].p0;
						lastp2=ilist[p].p1;
						lastp0=ilist[p].p2;
						break;

					    case 5:
						lastp0=ilist[p].p1;
						lastp1=ilist[p].p2;
						lastp2=ilist[p].p0;
						break;
					    }
					    stripindex[*nr_indices]=lastp0;				 //save
					    stripindex[(*nr_indices)+1]=lastp1;
					    stripindex[(*nr_indices)+2]=lastp2;
					    }
					if(i&1)                                  // odd or even, makes a difference, see OpenGL
					    {if (HitList[n].p[k].p0==lastp0&&HitList[n].p[k].p1==lastp2)    //new one fits ?
						{lastp0=HitList[n].p[k].p0;          // update last triangle used
						lastp1=HitList[n].p[k].p1;
						lastp2=HitList[n].p[k].p2;
						HitList[next].Stripped=TRUE;				 //this one is done
						stripindex[(*nr_indices)++]=lastp2;  //save, p2 defines this triangle
						n=next;                //use this one as next source
						k=10;									// break for-loop with k, found one
						i++;
						}	}
					else
					    {if (HitList[n].p[k].p0==lastp2&&HitList[n].p[k].p1==lastp1)    //new one fits ?
						{lastp0=HitList[n].p[k].p0;          // update last triangle used
						lastp1=HitList[n].p[k].p1;
						lastp2=HitList[n].p[k].p2;
						HitList[next].Stripped=TRUE;				 //this one is done
						if(!i++)
						    {
							(*nr_indices)+=3;								 //first triangle had 3 indices and keep it
							HitList[n].Stripped=TRUE;          //and is done also
						    }
						stripindex[(*nr_indices)++]=lastp2;  // save, p2 defines this triangle
						n=next;								//use this one as next source
						k=10;									// break for-loop with k, found one
						}	}	}	}	}
			    i++;														//actual number of triangles in strip is one more
			    if(i>1)strip_length[(*nr_strips)++]=i+2;		 //actual strip-length (in indices) is traingles +2
			}	}	}
    // Done.
    // Now keep all triangles that aren't stripped in original triangle-list and return
    // remaining number of triangles in original triangle-list.
    i=0;
    for(p=0;p<length;p++)
	{if(!HitList[p].Stripped)
	    {
		ilist[i++]=ilist[p];
	    }
	}
    GlobalFree(HitList);
    return(i);
}
