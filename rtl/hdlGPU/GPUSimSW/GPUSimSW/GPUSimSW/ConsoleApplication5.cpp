// ConsoleApplication5.cpp : Defines the entry point for the console application.
//

#include <stdio.h>

enum EMODE {
	CHECK_LEFT2RIGHT	= 1,
	CHECK_RIGHT2LEFT	= 2,
	CHECK_BOTH			= 3,
};

#define BIT_CNT		(4)
#define BIT_SIZE	(1<<BIT_CNT)
#define BIT_MSK		((BIT_SIZE)-1)

EMODE gmode = CHECK_BOTH;

void errorprint(const char* msg) {
	printf(msg);
	while (true) {

	}
}

// RESET TEST BUFFER TO DEFAULT STATUS
void clearBuff(int* buff) {
	for (int n = 0; n < 1024; n++) {
		buff[n] = n;
	}
}

// REFERENCE IMPLEMENTATION OF COPY.
void copyRef(int* bufferR, int from, int to, int l) {
	if (to <= from) {
		// Left to Right
		for (int v = 0; v < l; v++) {
			bufferR[(to + v) & 0x3FF] = bufferR[(from + v) & 0x3FF];
		}
	} else {
		// Right to left
		for (int v = l-1; v >= 0; v--) {
			bufferR[(to + v) & 0x3FF] = bufferR[(from + v) & 0x3FF];
		}
	}
}


// -----------------------------------------------------------------------
// HW ALGORITHM CONTEXT
// -----------------------------------------------------------------------
struct CPYCTX {
	int* adr;
	int  adrCount;

	int* buff;
	int from;					// DONE RegX0
	int to;						// DONE RegX1
	int l;						// DONE RegSizeW

	bool isLeftToRight;			// DONE xCopyDirectionIncr
	int  dir;					// DONE, same as xCopyDirectionIncr

	// LEFT TO RIGHT
	int sxm8;					// DONE RegX0[3:0]
	int dxm8;					// DONE RegX1[3:0]
	int idx;					// DONE cpyIdx

	bool dblLoad;				// DONE isDoubleLoad

	int tmpA[BIT_SIZE];			// TODO
	int tmpB[BIT_SIZE];			// TODO
	int mask[2];				// TODO (ctrl : use resetBank !)

	int curr;					// DONE cpyBank (ctrl : resetBank, switchBank) 
	int lastPixX;
	int sxe8;					// DONE sxe16
	int performSwitch;

	// 1024 / 16 pix = 64 block -> 6 bit.
	int blkfromS;				// DONE HW Use RELATIVE COUNTER and isLeftToRight additions.
	int blkCurrS;
	int blkfromE;
	int blkCurrD;
	int blkEndD;

	// TODO : Mask management, src, dst and stencil...
	// TODO : CPY_W2 condition : 

	bool read	(bool hasLength, bool hasLength2);
	void write	();
};

void copyImpl(CPYCTX& ctx);

// -----------------------------------------------------------------------
// [HW ALGORITHM IMPLEMENTATION]
// -----------------------------------------------------------------------

// SIMULATE 8 PIXEL READ BURST COMMAND
void read8(int* buff, int* dst, int blk8) {
	for (int n = 0; n < BIT_SIZE; n++) {
		dst[n] = buff[n + (blk8 << BIT_CNT)];
	}
}

// SIMULATE 8 PIXEL WRITE BURST COMMAND
void write8(int* buff, 
			int wrblk8, 
			int* old, 
			int* newB, 
			int maskOld, 
			int maskNew, 
			int maskWr, 
			int index) 
{
	for (int n = 0; n < BIT_SIZE; n++) {
		int v;
		int idxR = index + n;
		bool validP;

		if (idxR < BIT_SIZE) {
			v = old[idxR];
			validP = maskOld & (1 << idxR);
		} else {
			int idL = idxR - BIT_SIZE;
			v = newB[idL];
			validP = maskNew & (1 << idL);
		}

		if (validP && (maskWr & (1<<n))) {
			buff[(wrblk8 << BIT_CNT) + n] = v;
		}
	}
}

int getMaskLeft(int v) {
	switch (v) {
						 // Bit Order -> Pixel Order
	case 0: return 0xFFFF; // ####.####    ####.####
	case 1: return 0xFFFE; // ####.###      ###.####
	case 2: return 0xFFFC; // ####.##        ##.####
	case 3: return 0xFFF8; // ####.#          #.####
	case 4: return 0xFFF0; // ####.            .####
	case 5: return 0xFFE0; // ### .            . ###
	case 6: return 0xFFC0; // ##  .            .  ##
	case 7: return 0xFF80; // #   .            .   #
	case 8: return 0xFF00; // ####.####    ####.####
	case 9: return 0xFE00; // ####.###      ###.####
	case 10:return 0xFC00; // ####.##        ##.####
	case 11:return 0xF800; // ####.#          #.####
	case 12:return 0xF000; // ####.            .####
	case 13:return 0xE000; // ### .            . ###
	case 14:return 0xC000; // ##  .            .  ##
	case 15:return 0x8000; // #   .            .   #

	default:
		errorprint("invalid");
	}
}

int getMaskRight(int v) {
	switch (v) {
						 // Bit Order -> Pixel Order
	case 0: return 0x0001; //     .   #    #   .    
	case 1: return 0x0003; //     .  ##    ##  .     
	case 2: return 0x0007; //     . ###    ### . 
	case 3: return 0x000F; //     .####    ####.
	case 4: return 0x001F; //    #.####    ####.#
	case 5: return 0x003F; //   ##.####    ####.##
	case 6: return 0x007F; //  ###.####    ####.###
	case 7: return 0x00FF; // ####.####    ####.####
	case 8: return 0x01FF; //     .   #    #   .    
	case 9: return 0x03FF; //     .  ##    ##  .     
	case 10:return 0x07FF; //     . ###    ### . 
	case 11:return 0x0FFF; //     .####    ####.
	case 12:return 0x1FFF; //    #.####    ####.#
	case 13:return 0x3FFF; //   ##.####    ####.##
	case 14:return 0x7FFF; //  ###.####    ####.###
	case 15:return 0xFFFF; // ####.####    ####.####
	default:
		errorprint("invalid");
	}
}

bool CPYCTX::read(bool hasLength, bool hasLength2) {
	if (adr) {
		adr[adrCount++] = blkCurrS << (BIT_CNT+1);
	}

	read8(buff, curr ? tmpB : tmpA, blkCurrS);
	int msk;
	bool isLast	=  (blkCurrS == blkfromE);
	// Test not only start but also is NOT loop arriving at start.
	int mL		= getMaskLeft (sxm8);
	int mR		= getMaskRight(sxe8);
	msk			= ((blkCurrS == blkfromS) & !hasLength2) ? (isLeftToRight ? mL : mR) : 0xFFFF;
	msk		   &= isLast & (!hasLength)                  ? (isLeftToRight ? mR : mL) : 0xFFFF;

	bool allowNextRead = !isLast;

	mask[curr] = msk;

	// DONE HW
	blkCurrS += dir; blkCurrS &= (0x3FF >> BIT_CNT); // Loop, HW is just 'n' bit counter.
	// DONE HW
	curr ^= performSwitch;
	// DONE HW
	return allowNextRead | hasLength;
}

void CPYCTX::write() {
	if (adr) {
		adr[adrCount++] = 0x40000 | (blkCurrD << (BIT_CNT+1));
	}

	int  bank  = performSwitch & (curr ^ (!isLeftToRight));	// DONE HW
	write8(buff,
		blkCurrD,
		      bank ? tmpB : tmpA /* OLD taken, NEW swapped after READ !*/,
		      bank ? tmpA : tmpB,
		mask[ bank],			// OLD
		mask[!bank],		// NEW
		0xFFFF, /* TODO : TARGET MASKING PIXEL */
		idx
	);
	blkCurrD += dir; blkCurrD &= (0x3FF >> BIT_CNT); // Loop, HW is just 'n' bit counter.
}

void copyImpl(CPYCTX& ctx) {
	ctx.isLeftToRight = (ctx.to < ctx.from); // from > to easier in HW.

	ctx.dir  = ctx.isLeftToRight ? 1 : -1; // 01 vs 11 signed. { to > from , 1 }
	// LEFT TO RIGHT
	ctx.sxm8 = ctx.from & BIT_MSK;
	ctx.dxm8 = ctx.to   & BIT_MSK;

	ctx.curr = 0;
	ctx.lastPixX = (ctx.from + ctx.l - 1) & 0x3FF; // Loop, HW is just 'n' bit counter.
	ctx.sxe8     = ctx.lastPixX & BIT_MSK;

	int lPixX = ((((ctx.from & BIT_MSK) + (ctx.l & BIT_MSK)) & BIT_MSK) - 1) & BIT_MSK;

	if (lPixX != ctx.sxe8) {
		printf("ERROR");
	}

	// --- ATTENTION DEPENDANCY ORDER ---
	{	// DONE IN HW --------------------------
		bool dblLoadL2R = ctx.dxm8 < ctx.sxm8;
		ctx.idx = ((dblLoadL2R ? 0 /*TRUE*/ : BIT_SIZE /*FALSE*/) + ctx.sxm8 - ctx.dxm8) & 0xF;
		bool dblLoadR2L = ctx.sxe8 < ctx.idx;
		ctx.dblLoad = ctx.isLeftToRight ? dblLoadL2R : dblLoadR2L;
		// -------------------------------------
	}

	ctx.performSwitch = ((ctx.idx != 0) ? 1 : 0);

	int fromBS   = ctx.from     >> BIT_CNT;
	int fromBE   = ctx.lastPixX >> BIT_CNT;

	ctx.blkfromS = ctx.isLeftToRight ? fromBS : fromBE;
	ctx.blkfromE = ctx.isLeftToRight ? fromBE : fromBS;
	
	ctx.blkCurrS = ctx.blkfromS;

	int toBS     = ctx.to >> BIT_CNT;
	int toBE     = ((ctx.to + ctx.l - 1) & 0x3FF) >> BIT_CNT;

	ctx.blkCurrD = ctx.isLeftToRight ? toBS : toBE;
	ctx.blkEndD  = ctx.isLeftToRight ? toBE : toBS;

	// ------------------------------
	// PER LINE SETUP : HW DONE (Clear opposite on first read)
	ctx.mask[0] = 0;
	ctx.mask[1] = 0;

	// HW IGNORE
	if (ctx.isLeftToRight) {
		if ((gmode & EMODE::CHECK_LEFT2RIGHT) == 0) {
			copyRef(ctx.buff, ctx.from, ctx.to, ctx.l);
			return;
		}
	} else {
		if ((gmode & EMODE::CHECK_RIGHT2LEFT) == 0) {
			copyRef(ctx.buff, ctx.from, ctx.to, ctx.l);
			return;
		}
	}

	// Head, Read first block.
	// ------------------------------
	bool isLong = (ctx.l & 0x600); // >= 512 (1024 included !)	// HW DONE
	bool allowNextRead = ctx.read(isLong, false);				// HW DONE

	// ------------------------------
	// HW DONE
	if (ctx.dblLoad) {
		if (allowNextRead) {
			allowNextRead = ctx.read(isLong, isLong);
		} else {
			// Very small primitive. (Undo switch from first read)
			ctx.curr ^= 1; // -----DONE HW-----
		}
	}

	// ------------------------------
	// Loop // DONE HW
	while (allowNextRead) {
		ctx.write();
		allowNextRead = ctx.read(false,isLong);
	}
	// ------------------------------

	// Tail
	ctx.write();								// DONE HW

	// CLEAR MASK OLD !
	ctx.mask[ctx.curr] = 0;						// DONE HW : Embedded in previous write.
	// Swap BIT
	ctx.curr ^= ctx.performSwitch;					// TODO HW

	bool cond2 = (ctx.blkCurrD == ctx.blkEndD); // DONE HW
	// DONE HW
	if (cond2) {
		ctx.write();
	}
}

// ------------------------------------
// CALL TO REFERENCE AND HW ALGORITHM
// ------------------------------------
void copy(int* bufferR, int* bufferImpl, int from, int to, int l, int* adr, int* countAdr) {
	if (!adr) {
		if (from < 0 || from >= 1024) { errorprint("Invalid From"); }
		if (to < 0 || to >= 1024) { errorprint("Invalid To"); }

		copyRef(bufferR, from, to, l);
	}

	CPYCTX ctx;
	ctx.adr  = adr;
	ctx.buff = bufferImpl;
	ctx.from = from;
	ctx.to = to;
	ctx.l = l;
	ctx.adrCount = 0;
	copyImpl(ctx);
	if (adr) {
		*countAdr = ctx.adrCount;
	}
}

// ------------------------------------
// COMPARE BOTH BUFFER : TRUE IF DIFFERENCE.
// ------------------------------------
bool diffBuffer(int* bufferR, int* bufferImpl) {
	for (int n = 0; n < 1024; n++) {
		if (bufferImpl[n] != bufferR[n]) {
			return true;
		}
	}

	return false;
}

int bufferR   [1024];
int bufferImpl[1024];

// ------------------------------------
//  RUN A SINGLE TEST ON BOTH IMPLEMENTATION
// ------------------------------------

void test(int from, int to, int l, int* adr, int* adrCount) {
	printf("%i %i %i\n", from, to, l);

	if (l ==   0) { l = 1024; }
	if (l > 1024) { l = 1024; }

	int distFT = 1023;
	if (from > to) {
		// 255
		// 2
		distFT = to + 1024 - from;
	}

	/*
	if ((from > to) && (distFT < 16)) {
//		printf("REJECT : %i->%i (%i)\n", from, to, l);
		return;
	}
	*/
//	printf("Attempt : %i->%i (%i)\n", from, to, l);
	// Clear buffers
	if (!adr) {
		clearBuff(bufferR);
		clearBuff(bufferImpl);
	}

	// Run test on both.
	copy(bufferR, bufferImpl, from, to, l, adr, adrCount);

	if (!adr) {
		// Compare the results.
		if (diffBuffer(bufferR, bufferImpl)) {
			static int count = 0;
			if (from < to) {
				printf("ERR");
			}
			printf("[%i]Diff From:%4i To:%4i Len:%4i\n", count, from, to, l);
			count++;
			// errorprint("ERROR");
		}
	}
}

void test(int from, int to, int l) {
	test(from, to, l, NULL, NULL);
}

#include <stdlib.h>

void testLouis();

int mainOther()
{
	// testLouis();

	gmode = EMODE::CHECK_BOTH;
	/*
	int idx = 6;
	for (int x = 0; x < 100; x++) {
		for (int v = 1; v < 100; v++) {
			bool v1 = (((x + v - 1) % 8)       >= idx);
			bool v2 = ((((v % 8) + (x%8) - 1) %8) >= idx);
			printf("%i -> V1:%i , V2:%i\n", v, v1 ? 1 : 0, v2 ? 1 : 0);
		}
	}
	*/
	test(0, 16, 1);
	test(1, 17, 1);
	test(17, 1, 1);
	test(16, 0, 1);
	test(0, 0, 32);

//	test(0, 1008, 17);
	test(5, 7, 25); // PASS
	test(5, 7, 27); // PASS

	for (int d = 2; d < 200; d++) {
		test(1, d, 1); // PASS
	}
	
	test(  0,   1,   8);
	test(253,   0, 256);
	test(158,  90, 254); // PASS
	test(254, 244, 148); // PASS
	test(137,  28, 256); // PASS

	// Standard Test.

//	gmode = EMODE::CHECK_LEFT2RIGHT;
	// s = 0; d = 1008; l...

	for (int d = 0; d <= 1023; d++) {
		printf("--D=%i\n", d);
		for (int s = 0; s <= 1023; s++) {
			for (int l = 1; l <= 1024; l++) {
				test(s/*from*/, d, l);
			}
		}
	}

	/*
	srand(253487);
	for (int testRnd = 0; testRnd < 1000000; testRnd++) {
		int from = rand() & 0xFF;
		int   to = rand() & 0xFF;
		int    l = rand() & 0xFF;
		test(from, to, l);
	}
	*/

	return 0;
}

struct Element {
	const char* nom;
	int         prix;
	int         quantiteeMax;
};

void testLouis() {

	int sommeMax = 150000;

	const int ART_CNT = 17;

	Element articles[ART_CNT] = {
		{ "Marqueur pour tableau blanc",  6160, 0 },
		{ "Stylo Effaceur"             ,  2935, 0 },
		{ "Stick de colle 24g"         ,  4050, 0 },
		{ "Gomme Plastique"            ,  1327, 0 },
		{ "Regle Plate"                , 18300, 0 },
		{ "Ruban Scotch"               , 13490, 0 },
		{ "Boite de dix feutres"       , 46700, 0 },
		{ "Carte 10 aimants 9mm"       , 14290, 0 },
		{ "Boite de 10 disquettes"     , 58000, 0 },
		{ "Boite de 100 badges a ep."  ,121700, 0 },
		{ "Boite de 100 feuille A4"    , 68870, 0 },
		{ "Boite de 100 feuille postit", 10660, 0 },
		{ "Boite de 1000 enveloppes"   ,102500, 0 },
		{ "Cahier 96 pages"            ,  1390, 0 },
		{ "Cahier 100 pages"           ,  2483, 0 },
		{ "Boite de 1000 trombone"     ,  7900, 0 },
		{ "Boite de 100 attaches"      ,  8860, 0 },
	};

	int qte[ART_CNT] = {17, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 15, 0, 1, 0};
	int mySum = 0;
	for (int n = 0; n < ART_CNT; n++) {
		mySum += articles[n].prix * qte[n];
	}
	printf("%i\n", mySum);

	int sum		[ART_CNT];
	int solution[ART_CNT];
	long long int combinaison	= 0;
	int lastMax		= 0;

	for (int n = 0; n < ART_CNT; n++) {
		articles[n].quantiteeMax = sommeMax / articles[n].prix;
	}

	for (int m0 = 0; m0 <= articles[0].quantiteeMax; m0++) {
		qte[0] = m0;
		sum[0] = m0 * articles[0].prix;
		printf("M0 : %i\n",m0);
		for (int m1 = 0; m1 <= articles[1].quantiteeMax; m1++) {
			qte[1] = m1;
			sum[1] = sum[0] + m1 * articles[1].prix;
			if (sum[1] <= sommeMax)
			for (int m2 = 0; m2 <= articles[2].quantiteeMax; m2++) {
				qte[2] = m2;
				sum[2] = sum[1] + m2 * articles[2].prix;
				if (sum[1] <= sommeMax)
				for (int m3 = 0; m3 <= articles[3].quantiteeMax; m3++) {
					qte[3] = m3;
					sum[3] = sum[2] + m3 * articles[3].prix;
					if (sum[3] <= sommeMax)
					for (int m4 = 0; m4 <= articles[4].quantiteeMax; m4++) {
						qte[4] = m4;
						sum[4] = sum[3] + m4 * articles[4].prix;
						if (sum[4] <= sommeMax)
						for (int m5 = 0; m5 <= articles[5].quantiteeMax; m5++) {
							qte[5] = m5;
							sum[5] = sum[4] + m5 * articles[5].prix;
							if (sum[5] <= sommeMax)
							for (int m6 = 0; m6 <= articles[6].quantiteeMax; m6++) {
								qte[6] = m6;
								sum[6] = sum[5] + m6 * articles[6].prix;
								if (sum[6] <= sommeMax)
								for (int m7 = 0; m7 <= articles[7].quantiteeMax; m7++) {
									qte[7] = m7;
									sum[7] = sum[6] + m7 * articles[7].prix;
									if (sum[7] <= sommeMax)
									for (int m8 = 0; m8 <= articles[8].quantiteeMax; m8++) {
										qte[8] = m8;
										sum[8] = sum[7] + m8 * articles[8].prix;
										if (sum[8] <= sommeMax)
										for (int m9 = 0; m9 <= articles[9].quantiteeMax; m9++) {
											qte[9] = m9;
											sum[9] = sum[8] + m9 * articles[9].prix;
											if (sum[8] <= sommeMax)
											for (int m10 = 0; m10 <= articles[10].quantiteeMax; m10++) {
												qte[10] = m10;
												sum[10] = sum[9] + m10 * articles[10].prix;
												if (sum[10] <= sommeMax)
												for (int m11 = 0; m11 <= articles[11].quantiteeMax; m11++) {
													qte[11] = m11;
													sum[11] = sum[10] + m11 * articles[11].prix;
													if (sum[11] <= sommeMax)
													for (int m12 = 0; m12 <= articles[12].quantiteeMax; m12++) {
														qte[12] = m12;
														sum[12] = sum[11] + m12 * articles[12].prix;
														if (sum[12] <= sommeMax)
														for (int m13 = 0; m13 <= articles[13].quantiteeMax; m13++) {
															qte[13] = m13;
															sum[13] = sum[12] + m13 * articles[13].prix;
															if (sum[13] <= sommeMax)
															for (int m14 = 0; m14 <= articles[14].quantiteeMax; m14++) {
																qte[14] = m14;
																sum[14] = sum[13] + m14 * articles[14].prix;
																if (sum[14] <= sommeMax)
																for (int m15 = 0; m15 <= articles[15].quantiteeMax; m15++) {
																	qte[15] = m15;
																	sum[15] = sum[14] + m15 * articles[15].prix;
																	if (sum[15] <= sommeMax)
																	for (int m16 = 0; m16 <= articles[16].quantiteeMax; m16++) {
																		qte[16] = m16;
																		sum[16] = sum[15] + m16 * articles[16].prix;

																		int somme = sum[16];
																		int somme2 = 0; for (int idx = 0; idx < ART_CNT; idx++) { somme2 += articles[idx].prix * qte[idx]; }
																		if (somme2 != somme) {
																			printf("ERROR");
																		}
																		if (somme <= sommeMax) {
																			combinaison++;
																			if (somme >= lastMax) {
																			//	printf("Nouvelle solution %i : ", somme);
																				lastMax = somme;
																			/*	for (int cpy = 0; cpy < ART_CNT; cpy++) { printf("%i,", qte[cpy]);  solution[cpy] = qte[cpy]; }
																				printf("\n");
																			*/
																			}
																		}
																	}
																}
															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	printf("Combinaison : %i\n", combinaison);
	for (int n = 0; n < ART_CNT; n++) {
		printf("%i -> %i\n", solution[n]);
	}
}