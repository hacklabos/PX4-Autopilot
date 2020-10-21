#!/usr/bin/env python3

import nacl.encoding
import nacl.signing
import nacl.encoding
import nacl.hash
import struct
import binascii
import json
import time
import argparse
from pathlib import Path
import sys

def make_public_key_h_file(signing_key):
    """
    This file generate the public key header file
    to be included into the bootloader build.
    """
    public_key_c='\n'
    for i,c in enumerate(signing_key.verify_key.encode(encoder=nacl.encoding.RawEncoder)):
        public_key_c+= hex(c)
        public_key_c+= ', '
        if((i+1)%8==0):
            public_key_c+= '\n'
    with open("keyx.pub" ,mode='w') as f:
        f.write("//Public key to verify signed binaries")
        f.write(public_key_c)

def make_key_file(signing_key):
    """
    Writes the key.json file.
    Attention do not override your existing key files.
    Do not publish your private key!!
    """

    key_file = Path("keys.json")
    if key_file.is_file():
        print("ATTENTION: key.json already exists, are you sure you wnat to overwrite it?")
        print("Remove file and run script again.")
        print("Script aborted!")
        sys.exit(-1)

    keys={}
    keys["date"] = time.asctime()
    keys["public"] = (signing_key.verify_key.encode(encoder=nacl.encoding.HexEncoder)).decode()
    keys["private"] = binascii.hexlify(signing_key._seed).decode()
    #print (keys)
    with open("keys.json", "w") as write_file:
        json.dump(keys, write_file)
    return keys

def ed25519_sign(private_key, signee_bin):
    """
    This functino does the magic. It tkaes the pricate key and the binary file
    and generates the signed binary. it adds the meta data to the beginning of the file
    Ouput: "SignedBin.bin"
    """

    signing_key = nacl.signing.SigningKey(private_key, encoder=nacl.encoding.HexEncoder)

    # Sign a message with the signing key
    signed = signing_key.sign(signee_bin,encoder=nacl.encoding.RawEncoder)

    # Obtain the verify key for a given signing key
    verify_key = signing_key.verify_key

    # Serialize the verify key to send it to a third party
    verify_key_hex = verify_key.encode(encoder=nacl.encoding.HexEncoder)

    return signed.signature, verify_key_hex


def sign(bin_file_path, key_file_path=None):
    """
    reads the binary file and the key file.
    If the key file does not exist, it generates a
    new key file.
    """

    with open(bin_file_path,mode='rb') as f:
        signee_bin = f.read()
        # Align to 4 bytes. Signature always starts at
        # 4 byte aligned address, but the signee size
        # might not be aligned
        signee_bin += bytearray(b'\xff')*(4-len(signee_bin)%4)

    if key_file_path == None:
        key_file_path = 'keys.json'

    try:
        with open(key_file_path,mode='r') as f:
            keys = json.load(f)
        #print(keys)
    except:
        try:
            with open("Tools/test_keys.json",mode='r') as f:
                keys = json.load(f)
                print("WARNING: Signing with PX4 test key")
        except:
            print('Generating new key')
            keys=generate_key()

    signature, public_key = ed25519_sign(keys["private"], signee_bin)
    print("Binary \"%s\" signed."%bin_file_path)
    print("Signature:",binascii.hexlify(signature))
    print("Public key:",binascii.hexlify(public_key))

    return signee_bin + signature, public_key

def generate_key():
    """
    Call it and it generate two files,
    one file is made to be include in the bootloader build
    so its the "public_key.h" containg the verfication key.
    The other file key.json, containt both private and public key.
    Do not leak or loose the key file. This is mandatory for signing
    all future binaries you want to deploy!

    """

    # Generate a new random signing key
    signing_key = nacl.signing.SigningKey.generate()
     # Serialize the verify key to send it to a third party
    verify_key_hex = signing_key.verify_key.encode(encoder=nacl.encoding.HexEncoder)
    print("public key :",verify_key_hex)

    private_key_hex=binascii.hexlify(signing_key._seed)
    print("private key :",private_key_hex)

    keys = make_key_file(signing_key)
    make_public_key_h_file(signing_key)
    return keys

if(__name__ == "__main__"):

    parser = argparse.ArgumentParser(description="""CLI tool to calculate and add signature to px4. bin files\n
                                                  if given it takes an existing key file, else it generate new keys""",
                                    epilog="Output: SignedBin.bin and a key.json file")
    parser.add_argument("signee", help=".bin file to add signature")
    parser.add_argument("signed", help="signed output .bin", nargs='?', default=None)

    parser.add_argument("--key", help="key.json file", default=None)
    parser.add_argument("--rdct", help="binary R&D certificate file", default=None)
    args = parser.parse_args()

    # Sign the binary
    signed, public_key = sign(args.signee, args.key)

    with open(args.signed, mode='wb') as fs:
        # Write signed binary
        fs.write(signed)

    # Append rdcert if given
    try:
        with open(args.rdct ,mode='rb') as f:
            with open(args.signed, mode='ab') as fs:
                fs.write(f.read())
    except:
        pass

