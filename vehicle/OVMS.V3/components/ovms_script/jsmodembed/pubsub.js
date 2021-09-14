"use strict";var messages={},lastUid=-1;function hasKeys(b){var a;for(a in b){if(Object.prototype.hasOwnProperty.call(b,a)){return true}}return false}function callSubscriberWithImmediateExceptions(a,b,c){a(b,c)}function deliverMessage(a,c,d){var e=messages[c],b;if(!Object.prototype.hasOwnProperty.call(messages,c)){return}for(b in e){if(Object.prototype.hasOwnProperty.call(e,b)){callSubscriberWithImmediateExceptions(e[b],a,d)}}}function createDeliveryFunction(a,b){return function c(){var e=String(a),d=e.lastIndexOf(".");deliverMessage(a,a,b);while(d!==-1){e=e.substr(0,d);d=e.lastIndexOf(".");deliverMessage(a,e,b)}}}function messageHasSubscribers(c){var b=String(c),d=Boolean(Object.prototype.hasOwnProperty.call(messages,b)&&hasKeys(messages[b])),a=b.lastIndexOf(".");while(!d&&a!==-1){b=b.substr(0,a);a=b.lastIndexOf(".");d=Boolean(Object.prototype.hasOwnProperty.call(messages,b)&&hasKeys(messages[b]))}return d}function publish(b,c){b=(typeof b==="symbol")?b.toString():b;var d=createDeliveryFunction(b,c),a=messageHasSubscribers(b);if(!a){return false}d();return true}exports.publish=function(a,b){return publish(a,b)};exports.subscribe=function(c,b){if(typeof b!=="function"){return false}c=(typeof c==="symbol")?c.toString():c;if(!Object.prototype.hasOwnProperty.call(messages,c)){messages[c]={}}var a="uid_"+String(++lastUid);messages[c][a]=b;return a};exports.clearAllSubscriptions=function clearAllSubscriptions(){messages={}};exports.clearSubscriptions=function clearSubscriptions(b){var a;for(a in messages){if(Object.prototype.hasOwnProperty.call(messages,a)&&a.indexOf(b)===0){delete messages[a]}}};exports.unsubscribe=function(f){var b=function(k){var j;for(j in messages){if(Object.prototype.hasOwnProperty.call(messages,j)&&j.indexOf(k)===0){return true}}return false},e=typeof f==="string"&&(Object.prototype.hasOwnProperty.call(messages,f)||b(f)),c=!e&&typeof f==="string",a=typeof f==="function",i=false,d,h,g;if(e){exports.clearSubscriptions(f);return}for(d in messages){if(Object.prototype.hasOwnProperty.call(messages,d)){h=messages[d];if(c&&h[f]){delete h[f];i=f;break}if(a){for(g in h){if(Object.prototype.hasOwnProperty.call(h,g)&&h[g]===f){delete h[g];i=true}}}}}return i};exports.dump=function(){JSON.print(messages)};exports.data=function(){return messages};