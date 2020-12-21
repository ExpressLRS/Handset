
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "SimpleStore.h"

extern "C" {
#include "gd32vf103.h"
#include "systick.h"
}


// internal version of write that doesn't handle updates
// returns SS_NO_SPACE if there isn't room for the data
// returns SS_OK on success
// returns any error codes returned by addressOfFreespace()
uint8_t SimpleStore::_write(const uint16_t id, const uint16_t length, void *data)
{
    // check that there is enough space for the data (including any extra byte so that we can write half words)
    uint32_t spaceNeeded;
    // space needed for data may need rounding up if not a multiple of 2
    // include 4 bytes for id and length fields
    if (length % 2 != 0) {
        spaceNeeded = length + 5; // round up so we have a spare byte for writing the last value in a half word
    } else {
        spaceNeeded = length + 4;
    }

    if (spaceNeeded > freespaceRemaining()) {
        // not going to fit, return error code
        return SS_NO_SPACE;
    }

    // find the address to write the next object to
    uint32_t objAddr = addressOfFreespace();
    // we've already checked that there is enough space for the data, but might 
    // return SS_MISSING_EYECATCHER
    if ((uint32_t)objAddr < 256) {   // error codes are small values, valid addresses are large
        return (uint8_t) objAddr;
    }

    // printf("_write, writing obj at %lx\n\r", objAddr);

    // have to unlock to be able to write
    fmc_unlock();

    // write the id
    fmc_halfword_program(objAddr, id);

    // write the length
    fmc_halfword_program(objAddr+2, length);

    objAddr += 4; // objAddr is an int not a pointer, so we can move it in 1 byte units

    // from the examples. Presumably we should actually be checking for errors?
    fmc_flag_clear(FMC_FLAG_END);
    fmc_flag_clear(FMC_FLAG_WPERR);
    fmc_flag_clear(FMC_FLAG_PGERR);

    // write the data. We can only write words and halfwords. Fortunately we only write to the end of the storage
    // space, so we don't have to worry about overwriting the next entry.
    // For now, just write halfwords, we can worry about optimising for full words later.
    uint32_t halfWords = (length + 1) / 2;  // +1 to round up

    uint16_t * ptr = (uint16_t *) data;
    for(uint32_t i=0; i<halfWords; i++)
    {
        fmc_halfword_program(objAddr, *ptr);

        // from the examples. Presumably we should actually be checking for errors?
        // XXX does this need to be done for every write?
        fmc_flag_clear(FMC_FLAG_END);
        fmc_flag_clear(FMC_FLAG_WPERR);
        fmc_flag_clear(FMC_FLAG_PGERR);

        objAddr += 2;
        ptr++;  // pointer arithmetic for uint16_t *
    }

    // lock the fmc again to prevent unintentional writes
    fmc_lock();

    return SS_OK;
}

/** defragment the storage. Requires a page erase. Called from write().
 *  return SS_OK on success, SS_MISSING_EYECATCHER if storage not initiated,
 *    SS_NO_SPACE if the store is full of live objects.
 */
uint32_t SimpleStore::gc()
{
    uint8_t * p = (uint8_t *)SS_START_ADDRESS;

    // check eyecatcher
    if ( *((uint32_t *)p) != SS_EYECATCHER) {
        return SS_MISSING_EYECATCHER;
    }

    p += 4;

    uint32_t liveSize = sizeofLiveObjects();

    if (liveSize >= (SS_N_PAGES * FLASH_PAGE_SIZE - 4)) { // -4 to allow for the eyecatcher
        // gc isn't going to help so we might as well skip the wear and tear on the flash
        return SS_NO_SPACE;
    }

    uint8_t * copyBuffer = (uint8_t *)malloc(liveSize);

    if (copyBuffer == NULL) {
        printf("gc: failed to malloc buffer for %lu\n\r", liveSize);
        return SS_NO_SPACE;
    }

    // copy all the live records into the buffer
    uint8_t * copyPtr = copyBuffer;
    
    while(p < (uint8_t*)((SS_START_ADDRESS + (SS_N_PAGES * FLASH_PAGE_SIZE)) - 1)) 
    {
        // Is the current object in the freespace? All real objects have a uid != 0xFFFF
        if (*((uint16_t *)p) == 0xFFFF) {
            break;  // leave the loop
        }

        // read the length so we can adjust the pointer to the next object
        uint16_t objLength = *((uint16_t *)(p+2));

        // records are aligned to even addresses, so round up if necessary
        if (objLength % 2 != 0) objLength++;

        objLength += 4; // +4 to skip the id and length of this object

        // is the current object live?
        if (*((uint16_t *)p) != 0) {
            // copy it to the buffer
            memcpy(copyPtr, p, objLength);
            copyPtr += objLength;
        }

        p += objLength; 
    }

    // printf("gc collected %lu bytes of live records\n\r", ((uint32_t)copyPtr - (uint32_t)copyBuffer));

    fmc_unlock();

    // erase the page
    fmc_page_erase(SS_START_ADDRESS);

    // write the eyecatcher
    fmc_word_program(SS_START_ADDRESS, SS_EYECATCHER);

    // write all the records back to storage
    uint32_t nHalfwords = ((copyPtr - copyBuffer) + 1)/2;
    uint32_t destAddr = SS_START_ADDRESS + 4;
    copyPtr = copyBuffer;
    for (uint32_t i=0; i < nHalfwords; i++) 
    {
        fmc_halfword_program(destAddr, *((uint16_t*)copyPtr));
        destAddr += 2;
        copyPtr += 2;
    }

    fmc_lock();

    // free the copy buffer
    free(copyBuffer);

    return SS_OK;
}

// read the specified object from the storage area
uint8_t SimpleStore::read(const uint16_t id, uint16_t expectedLength, void *data)
{
    uint32_t objAddr = addressOf(id);

    // valid addresses are large, error codes are small
    if (objAddr < 256) {
        return (uint8_t)objAddr; // EARLY RETURN
    }

    // printf("read: reading object at %lx\n\r", objAddr);

    // validate the length matches the space we've been provided to return the data in
    uint16_t storedLen = *((uint16_t*)(objAddr+2));
    if (storedLen != expectedLength) {
        return SS_LENGTH_MISMATCH;
    }

    // copy the data into the provided buffer
    memcpy(data, (void *)(objAddr + 4), storedLen);

    return SS_OK;
}

/** Create a new record with the given id
 * Will remove any existing record for that id, so can be used for updates
 * Will initialise the storage area on first use, provided that the memory is previously untouched
*/
uint8_t SimpleStore::write(const uint16_t id, const uint16_t length, void *data)
{
    uint8_t result = remove(id);

    if (result == SS_MISSING_EYECATCHER)
    {
        // check if the storage area is untouched
        if (storageIsClean()) {
            // write the eyecatcher to the first word
            fmc_unlock();
            fmc_word_program(SS_START_ADDRESS, SS_EYECATCHER);
            fmc_flag_clear(FMC_FLAG_END);
            fmc_flag_clear(FMC_FLAG_WPERR);
            fmc_flag_clear(FMC_FLAG_PGERR);
            fmc_lock();
        }
    } // missing eyecatcher - storage wasn't initialised

    return _write(id, length, data);
}

/** Remove the specified record
 * @return SS_OK on success, SS_NOT_FOUND if no record for that id present in the store,
 *         SS_MISSING_EYECATCHER if the store has not been initialised
 */
uint8_t SimpleStore::remove(const uint16_t id)
{
    // get the address of the object, if it exists
    uint32_t objAddr = addressOf(id);

    if (objAddr < 256) {    // return any error conditions to the caller
        return (uint8_t) objAddr;
    }

    // printf("removing obj at %lx\n\r", objAddr);

    // overwrite the id with reserved value 0x00

    // have to unlock to be able to write
    fmc_unlock();

    // write the id and length as a halfword
    fmc_state_enum res = fmc_halfword_program(objAddr, 0);

    if (res) {} // just to hush the compiler warning when the debug is commented out

    // printf("res %d\n\r", res);

    // from the examples. Presumably we should actually be checking for errors?
    fmc_flag_clear(FMC_FLAG_END);
    fmc_flag_clear(FMC_FLAG_WPERR);
    fmc_flag_clear(FMC_FLAG_PGERR);

    fmc_lock();

    // delay(10);

    // uint32_t firstWord = *((uint32_t*)objAddr);
    // printf("firstword after program 0x%lx\n\r", firstWord); // the id is in the lowest bytes

    return SS_OK;
}

/** return the address of the specified object
 *  @return address if the object is in the store, SS_NOT_FOUND or SS_MISSING_EYECATCHER
 */
uint32_t SimpleStore::addressOf(const uint16_t id)
{
    uint8_t *p = (uint8_t*)SS_START_ADDRESS;

    if (*(uint32_t*)p != SS_EYECATCHER) return SS_MISSING_EYECATCHER;

    p += 4;

    while(p < (uint8_t*)((SS_START_ADDRESS + (SS_N_PAGES * FLASH_PAGE_SIZE)) - 1)) 
    {
        // printf("%p id %u\n\r", p, *((uint16_t *)p));

        // Does the ID of the current object match the specified value?
        if ( *((uint16_t *)p) == id) {
            return (uint32_t)p; // EARLY RETURN
        }

        // have we reached the freespace? An unassigned id will contain 0xFF (the flash erase value)
        if ( *((uint16_t *)p) == 0xFF) {
            return SS_NOT_FOUND; // EARLY RETURN
        }

        // read the length so we can adjust the pointer to the next object
        uint16_t objlength = *((uint16_t *)(p+2));
        // printf("len %u\n\r", objlength);

        p += objlength + 4; // +4 to skip the id and length of this object

        // records are aligned to even addresses, so adjust p if necessary
        if ((uint32_t)p % 2 != 0) p++;
    }

    return SS_NOT_FOUND;
}

/** Find the start of unused space in the storage area
 * 
 * Walk the set of objects until we find one with the reserved ID 0xFF
 * The returned address will be aligned to an even (half-word) boundary.
 * 
 * @return the first free (even aligned) address in the storage area or 
 *   SS_NO_SPACE if no freespace available
 *   SS_MISSING_EYECATCHER if the eyecatcher wasn't found
 * 
 */
uint32_t SimpleStore::addressOfFreespace()
{
    uint8_t *p = (uint8_t*)SS_START_ADDRESS;

    if (*(uint32_t*)p != SS_EYECATCHER) return SS_MISSING_EYECATCHER;

    p += 4;

    while(p < (uint8_t*)((SS_START_ADDRESS + (SS_N_PAGES * FLASH_PAGE_SIZE)) - 1)) 
    {
        // Is the current object in the freespace? All real objects have a uid != 0xFF
        if (*((uint16_t *)p) == 0xFFFF) {
            return (uint32_t)p; // EARLY RETURN
        }

        // read the length so we can adjust the pointer to the next object
        uint16_t objlength = *((uint16_t *)(p+2));
        p += objlength + 4; // +4 to skip the id and length of this object

        // records are aligned to even addresses, so adjust p if necessary
        if ((uint32_t)p % 2 != 0) p++;
    }

    return SS_NO_SPACE;
}

// Return the number of bytes available for new entries in the store
uint32_t SimpleStore::freespaceRemaining()
{
    uint32_t freeStart = addressOfFreespace();
    uint32_t storeEnd = FLASH_PAGE_SIZE * SS_N_PAGES + SS_START_ADDRESS;
    uint32_t space = storeEnd - freeStart;
    return space;
}

/** total storage used by live objects
 *  Returns the number of bytes needed to hold the live objects, including the headers.
 *  Doesn't check for a valid eyecatcher at the start of the store.
 * 
 * There's a problem with returning error codes as the minimum valid result is 4 (or 5 if we
 * forbid 0 length objects), and this could easily overlap with the small unsigned ints used
 * everywhere else for return codes.
 */
uint32_t SimpleStore::sizeofLiveObjects()
{
    uint8_t *p = (uint8_t*)(SS_START_ADDRESS + 4); // +4 to skip the eyecatcher, which we're going to assume is ok

    uint32_t totalSize = 0;

    while(p < (uint8_t*)((SS_START_ADDRESS + (SS_N_PAGES * FLASH_PAGE_SIZE)) - 1)) 
    {
        // Is the current object in the freespace? All real objects have a uid != 0xFFFF
        if (*((uint16_t *)p) == 0xFFFF) {
            break;  // leave the loop
        }

        // read the length so we can adjust the pointer to the next object
        uint16_t objLength = *((uint16_t *)(p+2));

        // records are aligned to even addresses, so round up if necessary
        if (objLength % 2 != 0) objLength++;

        objLength += 4; // +4 to skip the id and length of this object

        // is the current object live?
        if (*((uint16_t *)p) != 0) {
            totalSize += objLength;
        }

        p += objLength; 
    }

    // might get here if the store was completely full
    return totalSize;
}

bool SimpleStore::storageIsClean()
{
    uint32_t * p = (uint32_t *) SS_START_ADDRESS;
    uint32_t nWords = SS_N_PAGES * FLASH_PAGE_SIZE / 4;

    while(nWords > 0) {
        if (*p != 0xFFFFFFFF) return false;
        p++; // nasty pointer arithmetic will increase the address by the size of the type in the pointer declaration
        nWords--;
    }

    // if we made it to here then everything must be 0xFF, which means untouched flash
    return true;
}

//===============================

void SimpleStore::test()
{
    printf("flash read, checking page is clean...\n\r");

    // read words from start address and check they're all 0xFFFFFFFF
    uint32_t *p = (uint32_t *) SS_START_ADDRESS;
    uint32_t badwords = 0;
    for(int i=0; i<(SS_N_PAGES*FLASH_PAGE_SIZE/4); i++)
    {

        if (*p != 0xFFFFFFFF) {
            badwords++;
        }
        p++;
    }

    printf("flash checked, non-FF words=%lu\n\r", badwords);

    // write the eyecatcher to the first word of storage (unless it's already there)

    printf("flash write test\n\r");

    if (SS_EYECATCHER != *((uint32_t*)SS_START_ADDRESS)) {
        // write a value to the first word of the storage area
        fmc_unlock();
        fmc_word_program(SS_START_ADDRESS, SS_EYECATCHER);

        // from the examples. Presumably we should actually be checking for errors?
        fmc_flag_clear(FMC_FLAG_END);
        fmc_flag_clear(FMC_FLAG_WPERR);
        fmc_flag_clear(FMC_FLAG_PGERR);

        fmc_lock();

        delay(1); // needed?

        // read back and see what we got
        uint32_t *p = (uint32_t *) SS_START_ADDRESS;

        uint32_t x = *p;

        if (x == SS_EYECATCHER) {
            printf("success!\n\r");
        } else {
            printf("read unexpected value %lu\n\r", x);
        }

    } else {
        printf("eyecatcher already in place\n\r");
    }

    uint32_t freeAddr = addressOfFreespace();
    printf("freeAddr %p\n\r", (void*)freeAddr);

    printf("storageIsClean returns %u\n\r", storageIsClean());

    printf("space remaining %lu\n\r", freespaceRemaining());

    printf("used %lu\n\r", sizeofLiveObjects());

    uint32_t testCounter;

    #define TEST_ID 1
    uint8_t result;

    // try and remove any existing entries for the test object
    printf("removing any existing copies of the test object\n\r");
    result = SS_OK;
    // since remove isn't working properly, don't make it loop yet
    // while(result == SS_OK) 
    {
        result = remove(TEST_ID);
        delay(1);
        switch (result) {
            case SS_OK:
                printf("removed an instance of test object\n\r");
                break;
            case SS_NOT_FOUND:
                printf("no instances found to remove\n\r");
                break;
            default:
                printf("remove returned %u\n\r", result);
        }
    }

    // try and read a stored object - shouldn't be there if the remove worked
    printf("trying to read object, expect not found\n\r");
    result = read(TEST_ID, 4, &testCounter);
    if (result == SS_OK) {
        printf("test object was found! value was %lx\n\r", testCounter);
    } else if (result == SS_NOT_FOUND) {
        printf("test object was not found\n\r");
    } else {
        printf("read returned %u\n\r", result);
    }

    // write an object
    printf("writing object, expect ok\n\r");
    testCounter = 0x12345678;
    result = write(TEST_ID, 4, &testCounter);
    switch (result) {
        case SS_OK:
          printf("write ok\n\r");
          break;
        default:
          printf("write returned %u\n\r", result);
    }

    delay(1);

    // try and read the object
    printf("reading object back, expect ok with value 0x12345678\n\r");
    result = read(TEST_ID, 4, &testCounter);
    if (result == SS_OK) {
        printf("test object was found! value was %lx\n\r", testCounter);
    } else if (result == SS_NOT_FOUND) {
        printf("test object was not found\n\r");
    } else {
        printf("read returned %u\n\r", result);
    }

    printf("used %lu\n\r", sizeofLiveObjects());


    // write the object with a new value
    printf("updating object with new value, expect ok\n\r");
    testCounter = 0x87654321;
    result = write(TEST_ID, 4, &testCounter);
    switch (result) {
        case SS_OK:
          printf("write ok\n\r");
          break;
        default:
          printf("write returned %u\n\r", result);
    }

    delay(1);

    // try and read the object
    printf("reading object back, expect ok with value 0x87654321\n\r");
    result = read(TEST_ID, 4, &testCounter);
    if (result == SS_OK) {
        printf("test object was found! value was %lx\n\r", testCounter);
    } else if (result == SS_NOT_FOUND) {
        printf("test object was not found\n\r");
    } else {
        printf("read returned %u\n\r", result);
    }

    printf("used %lu\n\r", sizeofLiveObjects());


    // remove the object
    printf("removing object, expect ok\n\r");
    result = remove(TEST_ID);
    switch (result) {
        case SS_OK:
          printf("remove ok\n\r");
          break;
        case SS_NOT_FOUND:
          printf("remove couldn't find the object\n\r");
          break;
        default:
          printf("remove returned %u\n\r", result);
    }

    delay(1);

    // try and read the object
    printf("reading object, expect NOT_FOUND\n\r");
    result = read(TEST_ID, 4, &testCounter);
    if (result == SS_OK) {
        printf("test object was found! value was %lx\n\r", testCounter);
    } else if (result == SS_NOT_FOUND) {
        printf("test object has gone - congrats :)\n\r");
    } else {
        printf("read returned %u\n\r", result);
    }

    printf("used %lu\n\r", sizeofLiveObjects());
    printf("space remaining %lu\n\r", freespaceRemaining());

    // write a couple of new records
    printf("writing 2 records\n\r");

    testCounter = 0x11111111;
    write(2, 4, &testCounter);

    testCounter = 0x22222222;
    write(TEST_ID, 4, &testCounter);

    printf("used %lu\n\r", sizeofLiveObjects());
    printf("space remaining %lu\n\r", freespaceRemaining());

    printf("calling gc\n\r");
    gc();

    printf("used %lu\n\r", sizeofLiveObjects());
    printf("space remaining %lu\n\r", freespaceRemaining());

    printf("reading objects\n");

    // try and read the object
    printf("reading object, expect 0x11111111\n\r");
    result = read(2, 4, &testCounter);
    if (result == SS_OK) {
        printf("test object was found value was %lx\n\r", testCounter);
    } else if (result == SS_NOT_FOUND) {
        printf("test object not found\n\r");
    } else {
        printf("read returned %u\n\r", result);
    }

    // try and read the object
    printf("reading object, expect 0x22222222\n\r");
    result = read(TEST_ID, 4, &testCounter);
    if (result == SS_OK) {
        printf("test object was found value was %lx\n\r", testCounter);
    } else if (result == SS_NOT_FOUND) {
        printf("test object not found\n\r");
    } else {
        printf("read returned %u\n\r", result);
    }


}
