import createHash from "create-hash";
//import CryptoJS from 'crypto-js';
import forge from 'node-forge';

/** @arg {string|Buffer} data
    @arg {string} [digest = null] - 'hex', 'binary' or 'base64'
    @return {string|Buffer} - Buffer when digest is null, or string
*/
function sha256(data, encoding) {
    return createHash("sha256")
        .update(data)
        .digest(encoding);
}


export function  user_credentials_to_key(user_email, user_password) {
    const password_hash = sha256(user_password, 'base64');
    return 'credentials:'
      + user_email.length + "." + user_email + ","
      + password_hash.length + "." + password_hash;
}

export function symmetric_key_from_password(user_password){
    // const salt = CryptoJS.enc.Hex.parse('dd04ee6a8ad646719b4f9afbc7f673f8');
    // const result = CryptoJS.PBKDF2(user_password, salt, { hasher: CryptoJS.algo.SHA512, keySize: 256/32, iterations: 1000 });
    const result = forge.pkcs5.pbkdf2(user_password, forge.util.hexToBytes('dd04ee6a8ad646719b4f9afbc7f673f8'), 1000, 32, forge.md.sha512.create());
    return result;
}

export function decrypt_private_key(private_key, user_password){
    // const data = CryptoJS.enc.Base64.parse(private_key);
    // const result = CryptoJS.AES.decrypt({ciphertext: data}, key, {iv: CryptoJS.enc.Hex.parse('00000000000000000000000000000000'), mode: CryptoJS.mode.CBC, padding: CryptoJS.pad.Pkcs7});

    const key = symmetric_key_from_password(user_password);

    var decipher = forge.cipher.createDecipher('AES-CBC', key);
    decipher.start({iv: forge.util.hexToBytes('00000000000000000000000000000000')});
    const pk = forge.util.decode64(private_key);
    decipher.update(forge.util.createBuffer(pk, 'raw'));
    if(!decipher.finish()){
        throw "Decrypt error";
    }

    const asn = forge.asn1.fromDer(decipher.output);
    const result = forge.pki.privateKeyFromAsn1(asn);

    return result;
}

export function parse_public_key(public_key){
    const result = forge.pki.publicKeyFromPem(public_key);
    return result;
}

export function public_key_fingerprint(public_key){
    const result = forge.ssh.getPublicKeyFingerprint(public_key, { md: forge.md.sha256.create()});
    return result.getBytes();

}

export function hash_sha256(data){
    var md = forge.md.sha256.create();
    md.start();
    md.update(data, 'raw');
    const result = md.digest();
    return result.getBytes();
}

export function base64(data){
    const result = forge.util.encode64(data);
    return result;
}

export function from_hex(data){
    return forge.util.hexToBytes(data);
}

export function create_buffer(data){
    return new forge.util.ByteStringBuffer(data);
}

export function decode64(data){
    const result = forge.util.decode64(data);
    return result;
}

export function decrypt_by_private_key(private_key, message){
    var result = private_key.decrypt(message);
    return result;
}

export function buffer_read_number(buffer) {
    let value = buffer.getByte();
    
    if(0x80 > value){
      return value;
    }
    
    let result = 0;
    for(let i = (value & 0x7F); i > 0; --i){
        value = buffer.getByte();
        result <<= 8;
        result |= value;
    }
    
    return result + 0x80;  
}

export function buffer_pop_data(buffer) {
    const len = buffer_read_number(buffer);
    return buffer.getBytes(len);
}

export function buffer_get_string(buffer) {
    const data = buffer_pop_data(buffer);
    return forge.util.decodeUtf8(data);
}

export function decrypt_by_aes_256_cbc(key_seriliazed_data, message){
    const key_data = forge.util.createBuffer(key_seriliazed_data, 'raw');
    
    const key = buffer_pop_data(key_data);
    const iv = buffer_pop_data(key_data);

    const decipher = forge.cipher.createDecipher('AES-CBC', key);
    decipher.start({iv:iv});
    decipher.update(forge.util.createBuffer(message, 'raw'));
    if(!decipher.finish()){
        throw "Decrypt error";
    }

    return decipher.output;
}

export function crypt_by_aes_256_cbc(key, iv, message){
    const cipher = forge.cipher.createCipher('AES-CBC', key);
    cipher.start({iv:iv});
    cipher.update(forge.util.createBuffer(message, 'raw'));
    if(!cipher.finish()){
        throw "Crypt error";
    }

    return cipher.output.data;
}