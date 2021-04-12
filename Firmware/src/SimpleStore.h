#pragma once

/**
 * Minimalist persistent storage for GD32
 * 
 * Requirements:
 *   save a small number of data objects to persistent storage
 *   retrieve a specified data object
 *   update a specified data object with new values
 *   delete a specified data object from the store
 *   detect if the storage space is not valid so that we don't overwrite/erase code
 * 
 * Specifications:
 *   max ? objects (16 bit id with 2 reserved values puts upper limit at 65534, but there 
 *                  won't be enough ram to work with that many)
 *   max 255 bytes per object
 * 
 * Implementation:
 *   Underlying capabilities provide page erase to set all bits to 1 and
 *   write word or half word (no write byte?) via API calls.
 *   Pages are 1K
 *   Writes can only clear bits, so reserve 0 as a special marker.
 *     NB overwriting is only permitted if the new value is all 0, so we can't use read/modify/write to clear
 *     specific bits. Since the smallest unit for writes is 16 bits and we need to preserve the record length
 *     when deleting, id will have to have its own 16 field.
 *   reads can be performed using normal memory access (no api calls needed)
 *   use a simple eyecatcher in the first word of storage to validate that we're looking at the right memory.
 *   It's probably safe to auto-init the storage area on first use if the entire space contains 0xFF. This
 *   would save having to figure out a first-time setup mechanism.
 * 
 *   Define an object storage record as:
 *     16 bits  ID (0 and 0xFFFF reserved)
 *     8 bits   data length n in bytes
 *     n bytes  data
 * 
 *   Each object stored must have a unique ID by which it can be retrieved. ID 0 is reserved
 *   to indicate the object is no longer in use. ID 0xFFFF is reserved to detect freespace.
 * 
 *   Object records are aligned to even addresses. This makes sure that any garbage byte written
 *   as a result of not having a byte sized write api won't cause problems with the next record.
 * 
 *   Finding a specific object or the start of free space will involve traversing the set of
 *   objects using the data length fields, so these must be preserved even on deleted objects.
 *   Updating an object will involve setting the ID of the old version to 0 and writing a new
 *   copy at the end of the list (in the unused space). Unused space will be detected by the
 *   ID having the reserved value of 0xFFFF (the value after the page is erased).
 *   When there isn't room to write a new value at the end of the list we will need to collect
 *   all valid objects into ram, erase the page and then write everything back (a GC of the storage).
 *   
 *   1 page should be enough in the initial instance
 * 
 *   Conceptual behaviour (doesn't map directly to the API):
 * 
 *   SAVE
 *     validate the eyecatcher
 *       if eyecatcher not found, but the storage area is untouched 0xFFs, auto-init
 *       if eyecatcher not found, and the storage area contains non-0xFF data, then fail the call with appropriate error code.
 *     Search the storage for the first unused space (requires walking the set of objects using the length field)
 *       Q. Error check for ID already present? Simpler API would have a unified SAVE/UPDATE function that could handle both cases
 *     if there is not enough space available for the new object even after GC
 *       return with error code for storage full
 *     if there is not enough space to write the new object
 *       GC
 *     write the id, length and object data
 * 
 *   READ
 *     validate the eyecatcher
 *     Search the storage for the specified ID
 *     read the length
 *     read the object data
 * 
 *   UPDATE
 *     validate the eyecatcher
 *     Search the storage for the specified ID (if not found behave as SAVE)
 *     read the length
 *     if the length is the same as the new data length
 *       read the data and compare with new values
 *       if he data is unchanged, return
 *     Overwrite the ID with 0 to indicate that the old record is no longer in use 
 *       (Q. postpone until after a successful write of the new data? If we have a failure between operations is it better to 
 *           lose the old data or have a duplicate entry?)
 *     As per SAVE, to write the new data
 *     
 *   DELETE
 *     validate the eyecatcher
 *     Search the storage for the specified ID
 *     if found, overwrite the ID with 0
 * 
 *   Garbage Collect (GC)
 *       validate the eyecatcher
 *       copy all existing objects into ram
 *       erase the storage
 *       write all objects back to flash
 * 
 *  Things that need to be decided:
 *      Are 0 length objects allowed?
 */

#include <stdint.h>

#define FLASH_PAGE_SIZE 1024

// Use the last n pages of the address space
#define SS_N_PAGES 1
#define SS_START_ADDRESS (0x08020000 - (SS_N_PAGES * FLASH_PAGE_SIZE))

// The first word of the storage space contains a special value so we can validate
// we're looking at the right page
#define SS_EYECATCHER 0x55AA55AA

// Not actually using this anywhere yet, so treat with suspicion in case I let it get stale
struct SS_Record_s {
    uint16_t id;
    uint16_t length;
    uint8_t data[1];   // the first of length data bytes
};
typedef SS_Record_s SS_Record_t;

enum SS_Result {
    SS_OK,
    SS_LENGTH_MISMATCH,
    SS_NOT_FOUND,
    SS_NO_SPACE,
    SS_MISSING_EYECATCHER,
    SS_STORAGE_CORRUPTED
};

class SimpleStore
{
private:
    static uint8_t _write(const uint16_t id, const uint16_t length, void *data); // internal version of write that doesn't handle updates
    static uint32_t gc();  // defragment the storage. Requires a page erase. Called from write().
    static bool storageIsClean(); // return true if the storage area contains only 0xFF in all bytes

public:
    static uint8_t read(const uint16_t id, uint16_t expectedLength, void *data);
    static uint8_t write(const uint16_t id, const uint16_t length, void *data); // can do both first write and subsequent updates
    static uint8_t remove(const uint16_t id);

    static uint32_t addressOf(const uint16_t id); // return the address of the specified object, or 0 if not found
    static uint32_t addressOfFreespace();
    static uint32_t freespaceRemaining();
    static uint32_t sizeofLiveObjects();   // total storage used by live objects

    static void test(); // some basic function tests used during development
};